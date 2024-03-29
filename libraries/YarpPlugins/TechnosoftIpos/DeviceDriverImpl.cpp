// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TechnosoftIposBase.hpp"

#include <functional>

#include <yarp/os/LogStream.h>
#include <yarp/os/Property.h>

#include "LogComponent.hpp"
#include "TechnosoftIposEmcy.hpp"

using namespace roboticslab;

// seconds
constexpr auto DEFAULT_SDO_TIMEOUT = 0.02;
constexpr auto DEFAULT_DRIVE_STATE_TIMEOUT = 2.0;

// -----------------------------------------------------------------------------

bool TechnosoftIposBase::open(yarp::os::Searchable & config)
{
    const auto * robotConfig = *reinterpret_cast<const yarp::os::Property * const *>(config.find("robotConfig").asBlob());

    const auto & commonGroup = robotConfig->findGroup("common-ipos");
    yarp::os::Property iposGroup;

    if (!commonGroup.isNull())
    {
        iposGroup.fromString(commonGroup.toString());
    }

    iposGroup.fromString(config.toString(), false); // override common options

    const auto & driverGroup = robotConfig->findGroup(iposGroup.find("driver").asString());
    const auto & motorGroup = robotConfig->findGroup(iposGroup.find("motor").asString());
    const auto & gearboxGroup = robotConfig->findGroup(iposGroup.find("gearbox").asString());
    const auto & encoderGroup = robotConfig->findGroup(iposGroup.find("encoder").asString());

    canId = config.check("canId", yarp::os::Value(0), "CAN node ID").asInt32(); // id-specific

    actualControlMode = VOCAB_CM_NOT_CONFIGURED;

    axisName = config.check("name", yarp::os::Value(""), "axis name").asString(); // id-specific
    jointType = iposGroup.check("type", yarp::os::Value(yarp::dev::VOCAB_JOINTTYPE_UNKNOWN), "joint type [atrv|atpr|unkn]").asVocab32();
    max = iposGroup.check("max", yarp::os::Value(0.0), "max (meters or degrees)").asFloat64();
    min = iposGroup.check("min", yarp::os::Value(0.0), "min (meters or degrees)").asFloat64();
    maxVel = iposGroup.check("maxVel", yarp::os::Value(0.0), "maxVel (meters/second or degrees/second)").asFloat64();
    refSpeed = iposGroup.check("refSpeed", yarp::os::Value(0.0), "ref speed (meters/second or degrees/second)").asFloat64();
    refAcceleration = iposGroup.check("refAcceleration", yarp::os::Value(0.0), "ref acceleration (meters/second^2 or degrees/second^2)").asFloat64();
    drivePeakCurrent = driverGroup.check("peakCurrent", yarp::os::Value(0.0), "peak drive current (amperes)").asFloat64();
    k = motorGroup.check("k", yarp::os::Value(0.0), "motor constant").asFloat64();
    tr = gearboxGroup.check("tr", yarp::os::Value(0.0), "reduction").asFloat64();
    tr = tr * iposGroup.check("extraTr", yarp::os::Value(1.0), "extra reduction").asFloat64();
    encoderPulses = encoderGroup.check("encoderPulses", yarp::os::Value(0), "encoderPulses").asInt32();
    samplingPeriod = iposGroup.check("samplingPeriod", yarp::os::Value(0.0), "samplingPeriod (seconds)").asFloat64();
    reverse = iposGroup.check("reverse", yarp::os::Value(false), "reverse motor encoder counts").asBool();
    heartbeatPeriod = iposGroup.check("heartbeatPeriod", yarp::os::Value(0.0), "CAN heartbeat period (seconds)").asFloat64();
    syncPeriod = iposGroup.check("syncPeriod", yarp::os::Value(0.0), "SYNC message period (seconds)").asFloat64();
    // back-compat
    auto initialMode = iposGroup.check("initialMode", yarp::os::Value(VOCAB_CM_IDLE), "initial YARP control mode vocab").asVocab32();
    initialControlMode = iposGroup.check("initialControlMode", yarp::os::Value(initialMode), "initial YARP control mode vocab").asVocab32();

    if (!validateInitialState())
    {
        yCIError(IPOS, id()) << "Invalid configuration parameters";
        return false;
    }

    if (iposGroup.check("externalEncoder", "external encoder"))
    {
        std::string externalEncoder = iposGroup.find("externalEncoder").asString();
        yarp::os::Bottle & externalEncoderGroup = robotConfig->findGroup(externalEncoder);

        if (externalEncoderGroup.isNull())
        {
            yCIError(IPOS, id()) << "Missing external encoder device group" << externalEncoder;
            return false;
        }

        yarp::os::Property externalEncoderOptions;
        externalEncoderOptions.fromString(externalEncoderGroup.toString());
        externalEncoderOptions.put("robotConfig", config.find("robotConfig"));

        if (!externalEncoderDevice.open(externalEncoderOptions))
        {
            yCIError(IPOS, id()) << "Unable to open external encoder device" << externalEncoder;
            return false;
        }

        if (!externalEncoderDevice.view(iEncodersTimedRawExternal))
        {
            yCIError(IPOS, id()) << "Unable to view IEncodersTimedRaw in" << externalEncoder;
            return false;
        }

        if (!externalEncoderDevice.view(iExternalEncoderCanBusSharer))
        {
            yCIError(IPOS, id()) << "Unable to view ICanBusSharer in" << externalEncoder;
            return false;
        }
    }

    double sdoTimeout = iposGroup.check("sdoTimeout", yarp::os::Value(DEFAULT_SDO_TIMEOUT),
            "CAN SDO timeout (seconds)").asFloat64();
    double driveStateTimeout = iposGroup.check("driveStateTimeout", yarp::os::Value(DEFAULT_DRIVE_STATE_TIMEOUT),
            "CAN drive state timeout (seconds)").asFloat64();

    can = new CanOpenNode(canId, sdoTimeout, driveStateTimeout);

    // Manufacturer Status Register (1002h) and Modes of Operation Display (6061h)
    tpdo1Conf.addMapping<std::uint32_t>(0x1002).addMapping<std::int8_t>(0x6061);

    if (iposGroup.check("tpdo1InhibitTime", "TPDO1 inhibit time (seconds)"))
    {
        tpdo1Conf.setInhibitTime(iposGroup.find("tpdo1InhibitTime").asFloat64() * 1e4); // pass x100 microseconds
    }

    if (iposGroup.check("tpdo1EventTimer", "TPDO1 event timer (seconds)"))
    {
        tpdo1Conf.setEventTimer(iposGroup.find("tpdo1EventTimer").asFloat64() * 1e3); // pass milliseconds
    }

    // Motion Error Register (2000h) and Detailed Error Register (2002h)
    tpdo2Conf.addMapping<std::uint16_t>(0x2000).addMapping<std::uint16_t>(0x2002);

    if (iposGroup.check("tpdo2InhibitTime", "TPDO2 inhibit time (seconds)"))
    {
        tpdo2Conf.setInhibitTime(iposGroup.find("tpdo2InhibitTime").asFloat64() * 1e4); // pass x100 microseconds
    }

    if (iposGroup.check("tpdo2EventTimer", "TPDO2 event timer (seconds)"))
    {
        tpdo2Conf.setEventTimer(iposGroup.find("tpdo2EventTimer").asFloat64() * 1e3); // pass milliseconds
    }

    // Position actual internal value (6063h) and Torque actual value (6077h)
    tpdo3Conf.addMapping<std::int32_t>(0x6063).addMapping<std::int16_t>(0x6077);

    tpdo3Conf.setTransmissionType(PdoTransmissionType::SYNCHRONOUS_CYCLIC);

    using namespace std::placeholders;

    can->tpdo1()->registerHandler<std::uint16_t, std::uint16_t, std::int8_t>(std::bind(&TechnosoftIposBase::handleTpdo1, this, _1, _2, _3));
    can->tpdo2()->registerHandler<std::uint16_t, std::uint16_t>(std::bind(&TechnosoftIposBase::handleTpdo2, this, _1, _2));
    can->tpdo3()->registerHandler<std::int32_t, std::int16_t>(std::bind(&TechnosoftIposBase::handleTpdo3, this, _1, _2));

    can->emcy()->registerHandler(std::bind(&TechnosoftIposBase::handleEmcy, this, _1, _2, _3));
    can->emcy()->setErrorCodeRegistry<TechnosoftIposEmcy>();

    can->nmt()->registerHandler(std::bind(&TechnosoftIposBase::handleNmt, this, _1));

    if (iposGroup.check("monitorPeriod", "monitor thread period (seconds)"))
    {
        double monitorPeriod = iposGroup.find("monitorPeriod").asFloat64();

        if (monitorPeriod > 0.0)
        {
            monitorThread = new yarp::os::Timer(yarp::os::TimerSettings(monitorPeriod), std::bind(&TechnosoftIposBase::monitorWorker, this, _1), false);
        }
        else
        {
            heartbeatPeriod = 0.0; // disable
        }
    }

    return !monitorThread || monitorThread->start();
}

// -----------------------------------------------------------------------------

bool TechnosoftIposBase::close()
{
    // we need to do this in finalize(), too, since the monitor thread could be
    // still requesting CAN transfers even after CAN RX/TX threads have been
    // closed in CanBusBroker::close()
    if (monitorThread && monitorThread->isRunning())
    {
        monitorThread->stop();
    }

    delete monitorThread;
    monitorThread = nullptr;

    delete can;
    can = nullptr;

    if (externalEncoderDevice.isValid())
    {
        return externalEncoderDevice.close();
    }

    return true;
}

// -----------------------------------------------------------------------------
