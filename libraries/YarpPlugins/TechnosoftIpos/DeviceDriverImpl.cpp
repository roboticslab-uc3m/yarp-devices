// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TechnosoftIpos.hpp"

#include <functional>

#include <yarp/os/Property.h>

#include <ColorDebug.h>

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool TechnosoftIpos::open(yarp::os::Searchable & config)
{
    if (!config.check("robotConfig") || !config.find("robotConfig").isBlob())
    {
        CD_ERROR("Missing \"robotConfig\" property or not a blob.\n");
        return false;
    }

    const auto * robotConfig = *reinterpret_cast<const yarp::os::Property * const *>(config.find("robotConfig").asBlob());

    yarp::os::Bottle & commonGroup = robotConfig->findGroup("common-ipos");
    yarp::os::Property iposGroup;

    if (!commonGroup.isNull())
    {
        iposGroup.fromString(commonGroup.toString());
    }

    iposGroup.fromString(config.toString(), false); // override common options

    CD_DEBUG("%s\n", iposGroup.toString().c_str());

    yarp::os::Bottle & driverGroup = robotConfig->findGroup(iposGroup.find("driver").asString());
    yarp::os::Bottle & motorGroup = robotConfig->findGroup(iposGroup.find("motor").asString());
    yarp::os::Bottle & gearboxGroup = robotConfig->findGroup(iposGroup.find("gearbox").asString());
    yarp::os::Bottle & encoderGroup = robotConfig->findGroup(iposGroup.find("encoder").asString());

    int canId = config.check("canId", yarp::os::Value(0), "CAN node ID").asInt32(); // id-specific

    vars.actualControlMode = VOCAB_CM_NOT_CONFIGURED;

    vars.axisName = config.check("name", yarp::os::Value(""), "axis name").asString(); // id-specific
    vars.jointType = iposGroup.check("type", yarp::os::Value(yarp::dev::VOCAB_JOINTTYPE_UNKNOWN), "joint type [atrv|atpr|unkn]").asVocab();
    vars.max = iposGroup.check("max", yarp::os::Value(0.0), "max (meters or degrees)").asFloat64();
    vars.min = iposGroup.check("min", yarp::os::Value(0.0), "min (meters or degrees)").asFloat64();
    vars.maxVel = iposGroup.check("maxVel", yarp::os::Value(0.0), "maxVel (meters/second or degrees/second)").asFloat64();
    vars.refSpeed = iposGroup.check("refSpeed", yarp::os::Value(0.0), "ref speed (meters/second or degrees/second)").asFloat64();
    vars.refAcceleration = iposGroup.check("refAcceleration", yarp::os::Value(0.0), "ref acceleration (meters/second^2 or degrees/second^2)").asFloat64();
    vars.drivePeakCurrent = driverGroup.check("peakCurrent", yarp::os::Value(0.0), "peak drive current (amperes)").asFloat64();
    vars.k = motorGroup.check("k", yarp::os::Value(0.0), "motor constant").asFloat64();
    vars.tr = gearboxGroup.check("tr", yarp::os::Value(0.0), "reduction").asFloat64();
    vars.tr = vars.tr * iposGroup.check("extraTr", yarp::os::Value(1.0), "extra reduction").asFloat64();
    vars.encoderPulses = encoderGroup.check("encoderPulses", yarp::os::Value(0), "encoderPulses").asInt32();
    vars.pulsesPerSample = motorGroup.check("pulsesPerSample", yarp::os::Value(0), "pulsesPerSample").asInt32();
    vars.reverse = iposGroup.check("reverse", yarp::os::Value(false), "reverse motor encoder counts").asBool();
    vars.heartbeatPeriod = iposGroup.check("heartbeatPeriod", yarp::os::Value(0.0), "CAN heartbeat period (seconds)").asFloat64();
    vars.syncPeriod = iposGroup.check("syncPeriod", yarp::os::Value(0.0), "SYNC message period (seconds)").asFloat64();
    vars.initialMode = iposGroup.check("initialMode", yarp::os::Value(VOCAB_CM_IDLE), "initial YARP control mode vocab").asVocab();

    if (!vars.validateInitialState(canId))
    {
        CD_ERROR("Invalid configuration parameters.\n");
        return false;
    }

    if (iposGroup.check("externalEncoder", "external encoder"))
    {
        std::string externalEncoder = iposGroup.find("externalEncoder").asString();
        yarp::os::Bottle & externalEncoderGroup = robotConfig->findGroup(externalEncoder);

        if (externalEncoderGroup.isNull())
        {
            CD_ERROR("Missing external encoder device group %s.\n", externalEncoder.c_str());
            return false;
        }

        yarp::os::Property externalEncoderOptions;
        externalEncoderOptions.fromString(externalEncoderGroup.toString());
        externalEncoderOptions.put("robotConfig", config.find("robotConfig"));
        externalEncoderOptions.setMonitor(config.getMonitor(), externalEncoder.c_str());

        if (!externalEncoderDevice.open(externalEncoderOptions))
        {
            CD_ERROR("Unable to open external encoder device: %s.\n", externalEncoder.c_str());
            return false;
        }

        if (!externalEncoderDevice.view(iEncodersTimedRawExternal))
        {
            CD_ERROR("Unable to view IEncodersTimedRaw in %s.\n", externalEncoder.c_str());
            return false;
        }

        if (!externalEncoderDevice.view(iExternalEncoderCanBusSharer))
        {
            CD_ERROR("Unable to view ICanBusSharer in %s.\n", externalEncoder.c_str());
            return false;
        }
    }

    double sdoTimeout = iposGroup.check("sdoTimeout", yarp::os::Value(DEFAULT_SDO_TIMEOUT),
            "CAN SDO timeout (seconds)").asFloat64();
    double driveStateTimeout = iposGroup.check("driveStateTimeout", yarp::os::Value(DEFAULT_DRIVE_STATE_TIMEOUT),
            "CAN drive state timeout (seconds)").asFloat64();
    double monitorPeriod = iposGroup.check("monitorPeriod", yarp::os::Value(DEFAULT_MONITOR_PERIOD),
            "monitor thread period (seconds)").asFloat64();

    can = new CanOpen(canId, sdoTimeout, driveStateTimeout);

    std::uint16_t tpdo1InhibitTime = iposGroup.check("tpdo1InhibitTime", yarp::os::Value(0), "TPDO1 inhibit time (x100 microseconds)").asInt32();
    std::uint16_t tpdo2InhibitTime = iposGroup.check("tpdo2InhibitTime", yarp::os::Value(0), "TPDO2 inhibit time (x100 microseconds)").asInt32();

    std::uint16_t tpdo1EventTimer = iposGroup.check("tpdo1EventTimer", yarp::os::Value(0), "TPDO1 event timer (milliseconds)").asInt32();
    std::uint16_t tpdo2EventTimer = iposGroup.check("tpdo2EventTimer", yarp::os::Value(0), "TPDO2 event timer (milliseconds)").asInt32();

    PdoConfiguration tpdo1Conf;
    tpdo1Conf.addMapping<std::uint32_t>(0x1002).addMapping<std::int8_t>(0x6061);
    //tpdo1Conf.setInhibitTime(tpdo1InhibitTime); // TODO
    //tpdo1Conf.setEventTimer(tpdo1EventTimer); // TODO

    PdoConfiguration tpdo2Conf;
    tpdo2Conf.addMapping<std::uint16_t>(0x2000).addMapping<std::uint16_t>(0x2002);
    //tpdo2Conf.setInhibitTime(tpdo2InhibitTime); // TODO
    //tpdo2Conf.setEventTimer(tpdo2EventTimer); // TODO

    PdoConfiguration tpdo3Conf;
    tpdo3Conf.addMapping<std::int32_t>(0x6063).addMapping<std::int16_t>(0x6077);
    tpdo3Conf.setTransmissionType(PdoTransmissionType::SYNCHRONOUS_CYCLIC);

    vars.tpdo1Conf = tpdo1Conf;
    vars.tpdo2Conf = tpdo2Conf;
    vars.tpdo3Conf = tpdo3Conf;

    using namespace std::placeholders;

    can->tpdo1()->registerHandler<std::uint16_t, std::uint16_t, std::int8_t>(std::bind(&TechnosoftIpos::handleTpdo1, this, _1, _2, _3));
    can->tpdo2()->registerHandler<std::uint16_t, std::uint16_t>(std::bind(&TechnosoftIpos::handleTpdo2, this, _1, _2));
    can->tpdo3()->registerHandler<std::int32_t, std::int16_t>(std::bind(&TechnosoftIpos::handleTpdo3, this, _1, _2));

    can->emcy()->registerHandler(std::bind(&TechnosoftIpos::handleEmcy, this, _1, _2, _3));
    can->emcy()->setErrorCodeRegistry<TechnosoftIposEmcy>();

    can->nmt()->registerHandler(std::bind(&TechnosoftIpos::handleNmt, this, _1));

    monitorThread = new yarp::os::Timer(yarp::os::TimerSettings(monitorPeriod), std::bind(&TechnosoftIpos::monitorWorker, this, _1), true);

    return monitorThread->start();
}

// -----------------------------------------------------------------------------

bool TechnosoftIpos::close()
{
    if (monitorThread && monitorThread->isRunning())
    {
        monitorThread->stop();
    }

    delete monitorThread;
    monitorThread = nullptr;

    delete linInterpBuffer;
    linInterpBuffer = nullptr;

    delete can;
    can = nullptr;

    if (externalEncoderDevice.isValid())
    {
        return externalEncoderDevice.close();
    }

    return true;
}

// -----------------------------------------------------------------------------
