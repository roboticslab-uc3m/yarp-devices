// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TechnosoftIpos.hpp"

#include <functional>

#include <yarp/os/Network.h>
#include <yarp/os/Property.h>
#include <yarp/os/ResourceFinder.h>

#include <ColorDebug.h>

using namespace roboticslab;

namespace
{
    yarp::os::Property getConfig(const yarp::os::Searchable & config, const std::string & key, const std::string & comment,
            const std::string & dir)
    {
        yarp::os::ResourceFinder & rf = yarp::os::ResourceFinder::getResourceFinderSingleton();
        const std::string slash = yarp::os::NetworkBase::getPathSeparator();

        std::string value = config.check(key, yarp::os::Value(""), comment).asString();
        std::string path = rf.findFileByName(dir + slash + value + ".ini");

        yarp::os::Property nestedConfig;
        nestedConfig.setMonitor(config.getMonitor(), value.c_str());

        if (!nestedConfig.fromConfigFile(path))
        {
            CD_WARNING("File %s does not exist or unsufficient permissions.\n", path.c_str());
        }

        return nestedConfig;
    }
}

// -----------------------------------------------------------------------------

bool TechnosoftIpos::open(yarp::os::Searchable & config)
{
    CD_DEBUG("%s\n", config.toString().c_str());

    yarp::os::ResourceFinder & rf = yarp::os::ResourceFinder::getResourceFinderSingleton();
    const std::string slash = yarp::os::NetworkBase::getPathSeparator();

    int canId = config.check("canId", yarp::os::Value(0), "CAN node ID").asInt32();

    std::string joint = config.check("joint", yarp::os::Value(""), "controlled joint").asString();
    std::string jointPath = rf.findFileByName("joints/" + joint + ".ini");

    yarp::os::Property jointConfig = getConfig(config, "joint", "controlled joint", "joints");
    yarp::os::Property driverConfig = getConfig(jointConfig, "driver", "driver", "drivers");
    yarp::os::Property motorConfig = getConfig(jointConfig, "motor", "motor", "motors");
    yarp::os::Property transmissionConfig = getConfig(jointConfig, "transmission", "transmission", "transmissions");
    yarp::os::Property encoderConfig = getConfig(jointConfig, "encoder", "internal encoder", "encoders");

    // mutable variables
    vars.tr = transmissionConfig.check("tr", yarp::os::Value(0.0), "reduction").asFloat64();
    vars.k = driverConfig.check("k", yarp::os::Value(0.0), "motor constant").asFloat64();
    vars.encoderPulses = encoderConfig.check("encoderPulses", yarp::os::Value(0), "encoderPulses").asInt32();
    vars.pulsesPerSample = motorConfig.check("pulsesPerSample", yarp::os::Value(0), "pulsesPerSample").asInt32();

    vars.actualControlMode = VOCAB_CM_NOT_CONFIGURED;

    // immutable variables
    vars.drivePeakCurrent = driverConfig.check("drivePeakCurrent", yarp::os::Value(0.0), "peak drive current (amperes)").asFloat64();
    vars.maxVel = jointConfig.check("maxVel", yarp::os::Value(0.0), "maxVel (meters/second or degrees/second)").asFloat64();
    vars.axisName = jointConfig.check("axisName", yarp::os::Value(""), "axis name").asString();
    vars.jointType = jointConfig.check("jointType", yarp::os::Value(yarp::dev::VOCAB_JOINTTYPE_UNKNOWN), "joint type [atrv|atpr|unkn]").asVocab();
    vars.reverse = jointConfig.check("reverse", yarp::os::Value(false), "reverse motor encoder counts").asBool();
    vars.min = jointConfig.check("min", yarp::os::Value(0.0), "min (meters or degrees)").asFloat64();
    vars.max = jointConfig.check("max", yarp::os::Value(0.0), "max (meters or degrees)").asFloat64();
    vars.refSpeed = jointConfig.check("refSpeed", yarp::os::Value(0.0), "ref speed (meters/second or degrees/second)").asFloat64();
    vars.refAcceleration = jointConfig.check("refAcceleration", yarp::os::Value(0.0), "ref acceleration (meters/second^2 or degrees/second^2)").asFloat64();

    if (!vars.validateInitialState(canId))
    {
        CD_ERROR("Invalid configuration parameters.\n");
        return false;
    }

    if (config.check("externalEncoder", "external encoder"))
    {
        yarp::os::Property externalEncoderOptions = getConfig(config, "externalEncoder", "external encoder", "nodes");
        std::string externalEncoder = config.find("externalEncoder").asString();

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

    linInterpBuffer = LinearInterpolationBuffer::createBuffer(config);

    if (!linInterpBuffer)
    {
        return false;
    }

    double canSdoTimeoutMs = config.check("canSdoTimeoutMs", yarp::os::Value(0.0), "CAN SDO timeout (ms)").asFloat64();
    double canDriveStateTimeout = config.check("canDriveStateTimeout", yarp::os::Value(0.0), "CAN drive state timeout (s)").asFloat64();

    can = new CanOpen(canId, canSdoTimeoutMs * 0.001, canDriveStateTimeout);

    std::uint16_t tpdo1InhibitTime = config.check("tpdo1InhibitTime", yarp::os::Value(0), "TPDO1 inhibit time (x100 microseconds)").asInt32();
    std::uint16_t tpdo2InhibitTime = config.check("tpdo2InhibitTime", yarp::os::Value(0), "TPDO2 inhibit time (x100 microseconds)").asInt32();
    std::uint16_t tpdo3InhibitTime = config.check("tpdo3InhibitTime", yarp::os::Value(0), "TPDO3 inhibit time (x100 microseconds)").asInt32();

    std::uint16_t tpdo1EventTimer = config.check("tpdo1EventTimer", yarp::os::Value(0), "TPDO1 event timer (milliseconds)").asInt32();
    std::uint16_t tpdo2EventTimer = config.check("tpdo2EventTimer", yarp::os::Value(0), "TPDO2 event timer (milliseconds)").asInt32();
    std::uint16_t tpdo3EventTimer = config.check("tpdo3EventTimer", yarp::os::Value(0), "TPDO3 event timer (milliseconds)").asInt32();

    PdoConfiguration tpdo1Conf;
    tpdo1Conf.addMapping<std::uint32_t>(0x1002).addMapping<std::int8_t>(0x6061);
    tpdo1Conf.setInhibitTime(tpdo1InhibitTime);
    tpdo1Conf.setEventTimer(tpdo1EventTimer);

    PdoConfiguration tpdo2Conf;
    tpdo2Conf.addMapping<std::uint16_t>(0x2000).addMapping<std::uint16_t>(0x2002);
    tpdo2Conf.setInhibitTime(tpdo2InhibitTime);
    tpdo2Conf.setEventTimer(tpdo2EventTimer);

    PdoConfiguration tpdo3Conf;
    tpdo3Conf.addMapping<std::int32_t>(0x6063).addMapping<std::int16_t>(0x6077);
    tpdo3Conf.setInhibitTime(tpdo3InhibitTime);
    tpdo3Conf.setEventTimer(tpdo3EventTimer);

    vars.tpdo1Conf = tpdo1Conf;
    vars.tpdo2Conf = tpdo2Conf;
    vars.tpdo3Conf = tpdo3Conf;

    using namespace std::placeholders;

    can->tpdo1()->registerHandler<std::uint16_t, std::uint16_t, std::int8_t>(std::bind(&TechnosoftIpos::handleTpdo1, this, _1, _2, _3));
    can->tpdo2()->registerHandler<std::uint16_t, std::uint16_t>(std::bind(&TechnosoftIpos::handleTpdo2, this, _1, _2));
    can->tpdo3()->registerHandler<std::int32_t, std::int16_t>(std::bind(&TechnosoftIpos::handleTpdo3, this, _1, _2));

    can->emcy()->registerHandler(std::bind(&TechnosoftIpos::handleEmcy, this, _1, _2, _3));
    can->emcy()->setErrorCodeRegistry<TechnosoftIposEmcy>();

    CD_SUCCESS("CAN ID %d.\n", canId);
    return true;
}

// -----------------------------------------------------------------------------

bool TechnosoftIpos::close()
{
    CD_INFO("\n");

    delete linInterpBuffer;
    delete can;

    if (externalEncoderDevice.isValid())
    {
        return externalEncoderDevice.close();
    }

    return true;
}

// -----------------------------------------------------------------------------
