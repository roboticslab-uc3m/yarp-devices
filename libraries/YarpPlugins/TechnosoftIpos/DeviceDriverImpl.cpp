// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TechnosoftIpos.hpp"

#include <cstring>
#include <bitset>

#include <ColorDebug.h>

using namespace roboticslab;

namespace
{
    void interpretPtEmcy(uint16_t status, int canId, const LinearInterpolationBuffer * buffer)
    {
        CD_INFO("Interpolated position mode status. canId: %d.\n", canId);
        std::bitset<16> bits(status);

        if (bits.test(15))
        {
            CD_INFO("\t* buffer is empty.\n");

            if (buffer->getType() == "pvt")
            {
                if (bits.test(11))
                {
                    CD_INFO("\t* pvt maintained position on buffer empty (zero velocity).\n");
                }
                else
                {
                    CD_INFO("\t* pvt performed quick stop on buffer empty (non-zero velocity).\n");
                }
            }
        }
        else
        {
            CD_INFO("\t* buffer is not empty.\n");
        }

        if (bits.test(14))
        {
            CD_INFO("\t* buffer is low.\n");
        }
        else
        {
            CD_INFO("\t* buffer is not low.\n");
        }

        if (bits.test(13))
        {
            CD_INFO("\t* buffer is full.\n");
        }
        else
        {
            CD_INFO("\t* buffer is not full.\n");
        }

        if (bits.test(12))
        {
            CD_INFO("\t* integrity counter error.\n");
        }
        else
        {
            CD_INFO("\t* no integrity counter error.\n");
        }
    }
}

// -----------------------------------------------------------------------------

bool TechnosoftIpos::open(yarp::os::Searchable & config)
{
    CD_DEBUG("%s\n", config.toString().c_str());

    // -- .ini parameters (in order)
    int canId = config.check("canId",yarp::os::Value(0),"can bus ID").asInt32();
    this->maxVel = config.check("maxVel",yarp::os::Value(10),"maxVel (meters/second or degrees/second)").asFloat64();
    this->tr = config.check("tr",yarp::os::Value(0),"reduction").asFloat64();
    this->encoderPulses = config.check("encoderPulses",yarp::os::Value(0),"encoderPulses").asInt32();
    this->pulsesPerSample = config.check("pulsesPerSample",yarp::os::Value(0),"pulsesPerSample").asInt32();
    this->k = config.check("k",yarp::os::Value(0),"motor constant").asFloat64();

    // -- other parameters...
    this->modeCurrentTorque = VOCAB_CM_NOT_CONFIGURED;
    double canSdoTimeoutMs = config.check("canSdoTimeoutMs", yarp::os::Value(0.0), "CAN SDO timeout (ms)").asFloat64();
    double canDriveStateTimeout = config.check("canDriveStateTimeout", yarp::os::Value(0.0), "CAN drive state timeout (s)").asFloat64();

    double refAcceleration = config.check("refAcceleration",yarp::os::Value(0),"ref acceleration (meters/second^2 or degrees/second^2)").asFloat64();
    double refSpeed = config.check("refSpeed",yarp::os::Value(0),"ref speed (meters/second or degrees/second)").asFloat64();
    double max = config.check("max",yarp::os::Value(0),"max (meters or degrees)").asFloat64();
    double min = config.check("min",yarp::os::Value(0),"min (meters or degrees)").asFloat64();

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

    if( 0 == canId )
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
    if( 0 == this->pulsesPerSample )
    {
        CD_ERROR("Could not create TechnosoftIpos with pulsesPerSample 0\n");
        return false;
    }

    can = new CanOpen(canId, canSdoTimeoutMs * 0.001, canDriveStateTimeout);

    can->emcy()->setErrorCodeRegistry<TechnosoftIposEmcy>();

    can->emcy()->registerHandler([=](EmcyConsumer::code_t code, std::uint8_t reg, const std::uint8_t * msef)
            {
                if (code.first == 0xFF01)
                {
                    uint16_t status;
                    std::memcpy(&status, msef + 3, 2);
                    interpretPtEmcy(status, canId, linInterpBuffer);
                }
            });

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

    CD_SUCCESS("Created TechnosoftIpos with canId %d, tr %f, k %f, refAcceleration %f, refSpeed %f, encoderPulses %d, pulsesPerSample %d and all local parameters set to 0.\n",
               canId,tr,k,refAcceleration,refSpeed,encoderPulses,pulsesPerSample);
    return true;
}

// -----------------------------------------------------------------------------

bool TechnosoftIpos::close()
{
    CD_INFO("\n");

    bool ok = true;

    if (externalEncoderDevice.isValid())
    {
        ok &= externalEncoderDevice.close();
    }

    ok &= can->driveStatus()->requestTransition(DriveTransition::SWITCH_ON)
            && can->driveStatus()->requestTransition(DriveTransition::SHUTDOWN);

    delete linInterpBuffer;
    delete can;

    return ok;
}

// -----------------------------------------------------------------------------
