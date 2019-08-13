// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TechnosoftIpos.hpp"

// -----------------------------------------------------------------------------
bool roboticslab::TechnosoftIpos::open(yarp::os::Searchable& config)
{

    // -- .ini parameters (in order)
    this->canId = config.check("canId",yarp::os::Value(0),"can bus ID").asInt32();
    this->maxVel = config.check("maxVel",yarp::os::Value(10),"maxVel (meters/second or degrees/second)").asFloat64();
    this->tr = config.check("tr",yarp::os::Value(0),"reduction").asFloat64();
    this->encoderPulses = config.check("encoderPulses",yarp::os::Value(0),"encoderPulses").asInt32();
    this->k = config.check("k",yarp::os::Value(0),"motor constant").asFloat64();

    // -- other parameters...
    this->modeCurrentTorque = VOCAB_CM_NOT_CONFIGURED;
    double canSdoTimeoutMs = config.check("canSdoTimeoutMs", yarp::os::Value(0.0), "CAN SDO timeout (ms)").asFloat64();

    double refAcceleration = config.check("refAcceleration",yarp::os::Value(0),"ref acceleration (meters/second^2 or degrees/second^2)").asFloat64();
    double refSpeed = config.check("refSpeed",yarp::os::Value(0),"ref speed (meters/second or degrees/second)").asFloat64();
    double max = config.check("max",yarp::os::Value(0),"max (meters or degrees)").asFloat64();
    double min = config.check("min",yarp::os::Value(0),"min (meters or degrees)").asFloat64();

    linInterpBuffer = LinearInterpolationBuffer::createBuffer(config);

    if (!linInterpBuffer)
    {
        return false;
    }

    if( 0 == this->canId )
    {
        CD_ERROR("Could not create TechnosoftIpos with canId 0\n");
        return false;
    }
    if( min >= max )
    {
        CD_ERROR("Could not create TechnosoftIpos with min >= max\n");
        return false;
    }
    if( 0 == this->maxVel )
    {
        CD_ERROR("Could not create TechnosoftIpos with maxVel 0\n");
        return false;
    }
    if( 0 == this->tr )
    {
        CD_ERROR("Could not create TechnosoftIpos with tr 0\n");
        return false;
    }
    if( 0 == refAcceleration )
    {
        CD_ERROR("Could not create TechnosoftIpos with refAcceleration 0\n");
        return false;
    }
    if( 0 == refSpeed )
    {
        CD_ERROR("Could not create TechnosoftIpos with refSpeed 0\n");
        return false;
    }
    if( refSpeed > this->maxVel )
    {
        CD_ERROR("Could not create TechnosoftIpos with refSpeed > maxVel\n");
        return false;
    }
    if( 0 == this->encoderPulses )
    {
        CD_ERROR("Could not create TechnosoftIpos with encoderPulses 0\n");
        return false;
    }

    sdoSemaphore = new SdoSemaphore(canSdoTimeoutMs * 0.001);

    if (!setRefSpeedRaw(0, refSpeed))
    {
        CD_ERROR("Unable to set reference speed.\n");
        return false;
    }

    if (!setRefAccelerationRaw(0, refAcceleration))
    {
        CD_ERROR("Unable to set reference acceleration.\n");
        return false;
    }

    if (!setLimitsRaw(0, min, max))
    {
        CD_ERROR("Unable to set software limits.\n");
        return false;
    }

    CD_SUCCESS("Created TechnosoftIpos with canId %d, tr %f, k %f, refAcceleration %f, refSpeed %f, encoderPulses %d and all local parameters set to 0.\n",
               canId,tr,k,refAcceleration,refSpeed,encoderPulses);
    return true;
}

// -----------------------------------------------------------------------------
bool roboticslab::TechnosoftIpos::close()
{
    CD_INFO("\n");

    if (sdoSemaphore)
    {
        delete sdoSemaphore;
    }

    if (linInterpBuffer)
    {
        delete linInterpBuffer;
    }

    return true;
}

// -----------------------------------------------------------------------------
