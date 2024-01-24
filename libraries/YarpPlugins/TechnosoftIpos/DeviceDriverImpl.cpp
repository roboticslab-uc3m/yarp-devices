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
    yarp::os::Property iposOptions;
    iposOptions.fromString(config.findGroup("global").toString());
    iposOptions.fromString(config.findGroup("common").toString(), false); // override global options
    iposOptions.fromString(config.toString(), false); // override common options

    const auto & driverOptions = config.findGroup("driver");
    const auto & motorOptions = config.findGroup("motor");
    const auto & gearboxOptions = config.findGroup("gearbox");
    const auto & encoderOptions = config.findGroup("encoder");

    canId = config.check("canId", yarp::os::Value(0), "CAN node ID").asInt32(); // id-specific, don't override this

    actualControlMode = VOCAB_CM_NOT_CONFIGURED;

    axisName = config.check("name", yarp::os::Value(""), "axis name").asString(); // id-specific, don't override this
    jointType = iposOptions.check("type", yarp::os::Value(yarp::dev::VOCAB_JOINTTYPE_UNKNOWN), "joint type [atrv|atpr|unkn]").asVocab32();
    max = iposOptions.check("max", yarp::os::Value(0.0), "max (meters or degrees)").asFloat64();
    min = iposOptions.check("min", yarp::os::Value(0.0), "min (meters or degrees)").asFloat64();
    maxVel = iposOptions.check("maxVel", yarp::os::Value(0.0), "maxVel (meters/second or degrees/second)").asFloat64();
    refSpeed = iposOptions.check("refSpeed", yarp::os::Value(0.0), "ref speed (meters/second or degrees/second)").asFloat64();
    refAcceleration = iposOptions.check("refAcceleration", yarp::os::Value(0.0), "ref acceleration (meters/second^2 or degrees/second^2)").asFloat64();
    drivePeakCurrent = driverOptions.check("peakCurrent", yarp::os::Value(0.0), "peak drive current (amperes)").asFloat64();
    k = motorOptions.check("k", yarp::os::Value(0.0), "motor constant").asFloat64();
    tr = gearboxOptions.check("tr", yarp::os::Value(0.0), "reduction").asFloat64();
    tr = tr * iposOptions.check("extraTr", yarp::os::Value(1.0), "extra reduction").asFloat64();
    encoderPulses = encoderOptions.check("encoderPulses", yarp::os::Value(0), "encoderPulses").asInt32();
    samplingPeriod = iposOptions.check("samplingPeriod", yarp::os::Value(0.0), "samplingPeriod (seconds)").asFloat64();
    reverse = iposOptions.check("reverse", yarp::os::Value(false), "reverse motor encoder counts").asBool();
    heartbeatPeriod = iposOptions.check("heartbeatPeriod", yarp::os::Value(0.0), "CAN heartbeat period (seconds)").asFloat64();
    syncPeriod = iposOptions.check("syncPeriod", yarp::os::Value(0.0), "SYNC message period (seconds)").asFloat64();
    // back-compat
    auto initialMode = iposOptions.check("initialMode", yarp::os::Value(VOCAB_CM_IDLE), "initial YARP control mode vocab").asVocab32();
    initialControlMode = iposOptions.check("initialControlMode", yarp::os::Value(initialMode), "initial YARP control mode vocab").asVocab32();

    if (!validateInitialState())
    {
        yCIError(IPOS, id()) << "Invalid configuration parameters";
        return false;
    }

    double sdoTimeout = iposOptions.check("sdoTimeout", yarp::os::Value(DEFAULT_SDO_TIMEOUT),
            "CAN SDO timeout (seconds)").asFloat64();
    double driveStateTimeout = iposOptions.check("driveStateTimeout", yarp::os::Value(DEFAULT_DRIVE_STATE_TIMEOUT),
            "CAN drive state timeout (seconds)").asFloat64();

    can = new CanOpenNode(canId, sdoTimeout, driveStateTimeout);

    // Manufacturer Status Register (1002h) and Modes of Operation Display (6061h)
    tpdo1Conf.addMapping<std::uint32_t>(0x1002).addMapping<std::int8_t>(0x6061);

    if (iposOptions.check("tpdo1InhibitTime", "TPDO1 inhibit time (seconds)"))
    {
        tpdo1Conf.setInhibitTime(iposOptions.find("tpdo1InhibitTime").asFloat64() * 1e4); // pass x100 microseconds
    }

    if (iposOptions.check("tpdo1EventTimer", "TPDO1 event timer (seconds)"))
    {
        tpdo1Conf.setEventTimer(iposOptions.find("tpdo1EventTimer").asFloat64() * 1e3); // pass milliseconds
    }

    // Motion Error Register (2000h) and Detailed Error Register (2002h)
    tpdo2Conf.addMapping<std::uint16_t>(0x2000).addMapping<std::uint16_t>(0x2002);

    if (iposOptions.check("tpdo2InhibitTime", "TPDO2 inhibit time (seconds)"))
    {
        tpdo2Conf.setInhibitTime(iposOptions.find("tpdo2InhibitTime").asFloat64() * 1e4); // pass x100 microseconds
    }

    if (iposOptions.check("tpdo2EventTimer", "TPDO2 event timer (seconds)"))
    {
        tpdo2Conf.setEventTimer(iposOptions.find("tpdo2EventTimer").asFloat64() * 1e3); // pass milliseconds
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

    if (iposOptions.check("monitorPeriod", "monitor thread period (seconds)"))
    {
        double monitorPeriod = iposOptions.find("monitorPeriod").asFloat64();

        if (monitorPeriod > 0.0)
        {
            monitorThread = new yarp::os::Timer(yarp::os::TimerSettings(monitorPeriod), std::bind(&TechnosoftIposBase::monitorWorker, this, _1), false);
        }
        else
        {
            heartbeatPeriod = 0.0; // disable
        }
    }

    if (yarp::os::Value * val; iposOptions.check("disabledEncoders", val, "disabled encoder IDs") && val->isList())
    {
        const auto * ids = val->asList();

        for (int i = 0; i < ids->size(); i++)
        {
            disabledEncoderIds.push_back(ids->get(i).asInt32());
        }
    }

    return true;
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

    return true;
}

// -----------------------------------------------------------------------------
