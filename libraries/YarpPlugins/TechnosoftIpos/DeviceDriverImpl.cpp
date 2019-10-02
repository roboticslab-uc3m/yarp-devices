// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TechnosoftIpos.hpp"

#include <functional>

#include <ColorDebug.h>

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool TechnosoftIpos::open(yarp::os::Searchable & config)
{
    CD_DEBUG("%s\n", config.toString().c_str());

    int canId = config.check("canId", yarp::os::Value(0), "CAN bus ID").asInt32();

    // mutable variables
    vars.drivePeakCurrent = config.check("drivePeakCurrent", yarp::os::Value(0.0), "peak drive current (amperes)").asFloat64();
    vars.maxVel = config.check("maxVel", yarp::os::Value(0.0), "maxVel (meters/second or degrees/second)").asFloat64();
    vars.tr = config.check("tr", yarp::os::Value(0.0), "reduction").asFloat64();
    vars.k = config.check("k", yarp::os::Value(0.0), "motor constant").asFloat64();
    vars.encoderPulses = config.check("encoderPulses", yarp::os::Value(0), "encoderPulses").asInt32();
    vars.pulsesPerSample = config.check("pulsesPerSample", yarp::os::Value(0), "pulsesPerSample").asInt32();

    // immutable variables
    vars.min = config.check("min", yarp::os::Value(0.0), "min (meters or degrees)").asFloat64();
    vars.max = config.check("max", yarp::os::Value(0.0), "max (meters or degrees)").asFloat64();
    vars.refSpeed = config.check("refSpeed", yarp::os::Value(0.0), "ref speed (meters/second or degrees/second)").asFloat64();
    vars.refAcceleration = config.check("refAcceleration", yarp::os::Value(0.0), "ref acceleration (meters/second^2 or degrees/second^2)").asFloat64();
    vars.axisName = config.check("axisName", yarp::os::Value(""), "axis name").asString();
    vars.jointType = config.check("jointType", yarp::os::Value(yarp::dev::VOCAB_JOINTTYPE_UNKNOWN), "joint type [atrv|atpr|unkn]").asVocab();
    vars.reverse = config.check("reverse", yarp::os::Value(false), "reverse motor encoder counts").asBool();

    vars.actualControlMode = VOCAB_CM_NOT_CONFIGURED;

    if (!vars.validateInitialState(canId))
    {
        CD_ERROR("Invalid configuration parameters.\n");
        return false;
    }

    double canSdoTimeoutMs = config.check("canSdoTimeoutMs", yarp::os::Value(0.0), "CAN SDO timeout (ms)").asFloat64();
    double canDriveStateTimeout = config.check("canDriveStateTimeout", yarp::os::Value(0.0), "CAN drive state timeout (s)").asFloat64();

    if (config.check("externalEncoder", "external encoder device"))
    {
        std::string externalEncoder = config.find("externalEncoder").asString();

        if (!externalEncoderDevice.open(externalEncoder))
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

    can = new CanOpen(canId, canSdoTimeoutMs * 0.001, canDriveStateTimeout);

    PdoConfiguration tpdo1Conf; // TODO
    tpdo1Conf.addMapping<std::uint16_t>(0x6041);

    PdoConfiguration tpdo2Conf; // TODO
    tpdo2Conf.addMapping<std::int8_t>(0x6061);

    if (!can->tpdo1()->configure(tpdo1Conf))
    {
        CD_ERROR("Unable to configure TPDO1.\n");
        return false;
    }

    if (!can->tpdo2()->configure(tpdo2Conf))
    {
        CD_ERROR("Unable to configure TPDO2.\n");
        return false;
    }

    using namespace std::placeholders;

    can->tpdo1()->registerHandler<std::uint16_t>(std::bind(&TechnosoftIpos::interpretStatusword, this, _1));
    can->tpdo2()->registerHandler<std::int8_t>(std::bind(&TechnosoftIpos::interpretModesOfOperation, this, _1));
    can->emcy()->registerHandler(std::bind(&TechnosoftIpos::emcyCb, this, _1, _2, _3));
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
