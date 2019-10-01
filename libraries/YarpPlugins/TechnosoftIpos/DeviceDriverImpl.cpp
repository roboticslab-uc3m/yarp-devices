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

    vars.actualControlMode = VOCAB_CM_NOT_CONFIGURED;

    if (canId == 0)
    {
        CD_ERROR("Illegal CAN ID 0.\n");
        return false;
    }

    if (!vars.validateInitialState())
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

    can->tpdo1()->registerHandler<std::uint16_t>([=](std::uint16_t statusword)
            {
                can->driveStatus()->update(statusword);

                switch (can->driveStatus()->getCurrentState())
                {
                case DriveState::FAULT_REACTION_ACTIVE:
                case DriveState::FAULT:
                    vars.actualControlMode = VOCAB_CM_HW_FAULT;
                    break;
                case DriveState::SWITCHED_ON:
                    vars.actualControlMode = VOCAB_CM_IDLE;
                    break;
                }
            });

    can->tpdo2()->registerHandler<std::int8_t>([=](std::int8_t modesOfOperation)
            {
                switch (modesOfOperation)
                {
                // handled
                case -5:
                    CD_INFO("iPOS specific: External Reference Torque Mode. canId: %d.\n", can->getId());
                    vars.actualControlMode = vars.requestedcontrolMode == VOCAB_CM_TORQUE ? VOCAB_CM_TORQUE : VOCAB_CM_CURRENT;
                    break;
                case 1:
                    CD_INFO("Profile Position Mode. canId: %d.\n", can->getId());
                    vars.actualControlMode = VOCAB_CM_POSITION;
                    break;
                case 3:
                    CD_INFO("Profile Velocity Mode. canId: %d.\n", can->getId());
                    vars.actualControlMode = VOCAB_CM_VELOCITY;
                    break;
                case 7:
                    CD_INFO("Interpolated Position Mode. canId: %d.\n", can->getId());
                    vars.actualControlMode = VOCAB_CM_POSITION_DIRECT;
                    break;
                // unhandled
                case -4:
                    CD_INFO("iPOS specific: External Reference Speed Mode. canId: %d.\n", can->getId());
                    vars.actualControlMode = VOCAB_CM_UNKNOWN;
                    break;
                case -3:
                    CD_INFO("iPOS specific: External Reference Position Mode. canId: %d.\n", can->getId());
                    vars.actualControlMode = VOCAB_CM_UNKNOWN;
                    break;
                case -2:
                    CD_INFO("iPOS specific: Electronic Camming Position Mode. canId: %d.\n", can->getId());
                    vars.actualControlMode = VOCAB_CM_UNKNOWN;
                    break;
                case -1:
                    CD_INFO("iPOS specific: Electronic Gearing Position Mode. canId: %d.\n", can->getId());
                    vars.actualControlMode = VOCAB_CM_UNKNOWN;
                    break;
                case 6:
                    CD_INFO("Homing Mode. canId: %d.\n", can->getId());
                    vars.actualControlMode = VOCAB_CM_UNKNOWN;
                    break;
                case 8:
                    CD_INFO("Cyclic Synchronous Position Mode. canId: %d.\n", can->getId());
                    vars.actualControlMode = VOCAB_CM_UNKNOWN;
                    break;
                default:
                    CD_WARNING("Mode \"%d\" not specified in manual, may be in Fault or not enabled yet. canId: %d.\n", modesOfOperation, can->getId());
                    vars.actualControlMode = VOCAB_CM_UNKNOWN;
                    break;
                }
            });

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
