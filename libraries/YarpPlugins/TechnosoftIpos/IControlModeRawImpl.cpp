// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TechnosoftIpos.hpp"

#include <yarp/os/Vocab.h>

#include <ColorDebug.h>

#include "CanUtils.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool TechnosoftIpos::setPositionDirectModeRaw()
{
    CD_DEBUG("\n");

    linInterpBuffer->resetIntegrityCounter();

    PdoConfiguration rpdo3Conf;
    rpdo3Conf.addMapping<std::uint32_t>(0x60C1, 0x01);
    rpdo3Conf.addMapping<std::uint32_t>(0x60C1, 0x02);

    bool ok = true;
    ok = ok && can->rpdo3()->configure(rpdo3Conf);
    ok = ok && can->sdo()->download<std::int8_t>("Modes of Operation", 7, 0x6060);
    ok = ok && can->sdo()->download<std::int16_t>("Interpolation sub mode select", linInterpBuffer->getSubMode(), 0x60C0);
    ok = ok && can->sdo()->download<std::uint16_t>("Interpolated position buffer length", linInterpBuffer->getBufferSize(), 0x2073);
    ok = ok && can->sdo()->download<std::uint16_t>("Interpolated position buffer configuration", 0xA000, 0x2074);
    if (!ok) return false;

    double ref;
    if (!getEncoderRaw(0, &ref)) return false;
    std::int32_t data = vars.degreesToInternalUnits(ref);

    if (!can->sdo()->download("Interpolated position initial position", data, 0x2079))
    {
        return false;
    }

    yarp::os::Time::delay(0.1);  //-- Seems like a "must".

    linInterpBuffer->setInitialReference(ref);
    linInterpBuffer->updateTarget(ref);

    for (int i = 0; i < linInterpBuffer->getBufferSize(); i++)
    {
        if (!can->rpdo3()->write<std::uint64_t>(linInterpBuffer->makeDataRecord()))
        {
            CD_ERROR("Unable to send point %d/%d to buffer.\n", i + 1, linInterpBuffer->getBufferSize());
            return false;
        }
    }

    return true;
}

// -----------------------------------------------------------------------------

bool TechnosoftIpos::getControlModeRaw(int j, int * mode)
{
    //CD_DEBUG("(%d)\n", j); //-- Too verbose in controlboardwrapper2 stream
    CHECK_JOINT(j);
    *mode = vars.actualControlMode;
    return true;
}

// -----------------------------------------------------------------------------

bool TechnosoftIpos::getControlModesRaw(int * modes)
{
    CD_DEBUG("\n");
    return getControlModeRaw(0, &modes[0]);
}

// -----------------------------------------------------------------------------

bool TechnosoftIpos::getControlModesRaw(int n_joint, const int * joints, int * modes)
{
    CD_DEBUG("\n");
    return getControlModeRaw(joints[0], &modes[0]);
}

// -----------------------------------------------------------------------------

bool TechnosoftIpos::setControlModeRaw(int j, int mode)
{
    CD_DEBUG("(%d, %s)\n", j, yarp::os::Vocab::decode(mode).c_str());
    CHECK_JOINT(j);

    vars.requestedcontrolMode = mode;

    switch (mode)
    {
    case VOCAB_CM_POSITION:
        return can->driveStatus()->requestState(DriveState::OPERATION_ENABLED)
                && can->sdo()->download<std::int8_t>("Modes of Operation", 1, 0x6060)
                && vars.awaitControlMode(mode);

    case VOCAB_CM_VELOCITY:
        return can->driveStatus()->requestState(DriveState::OPERATION_ENABLED)
                && can->sdo()->download<std::int8_t>("Modes of Operation", 3, 0x6060)
                && vars.awaitControlMode(mode);

    case VOCAB_CM_CURRENT:
    case VOCAB_CM_TORQUE:
        return can->driveStatus()->requestState(DriveState::OPERATION_ENABLED)
                && can->sdo()->download<std::uint16_t>("External Reference Type", 1, 0x201D)
                && can->sdo()->download<std::int8_t>("Modes of Operation", -5, 0x6060)
                && can->rpdo1()->write<std::uint16_t>(0x001F)
                && vars.awaitControlMode(mode);

    case VOCAB_CM_POSITION_DIRECT:
        return can->driveStatus()->requestState(DriveState::OPERATION_ENABLED)
                && setPositionDirectModeRaw()
                && vars.awaitControlMode(mode);

    case VOCAB_CM_FORCE_IDLE:
        if (vars.actualControlMode == VOCAB_CM_HW_FAULT
                && !can->driveStatus()->requestTransition(DriveTransition::FAULT_RESET))
        {
            CD_ERROR("Unable to reset fault status.\n");
            return false;
        }

        // no break

    case VOCAB_CM_IDLE:
        return can->driveStatus()->requestState(DriveState::SWITCHED_ON);

    default:
        CD_ERROR("Unsupported, unknown or read-only mode: %s.\n", yarp::os::Vocab::decode(mode).c_str());
        return false;
    }
}

// -----------------------------------------------------------------------------

bool TechnosoftIpos::setControlModesRaw(int * modes)
{
    CD_DEBUG("\n");
    return setControlModeRaw(0, modes[0]);
}

// -----------------------------------------------------------------------------

bool TechnosoftIpos::setControlModesRaw(int n_joint, const int * joints, int * modes)
{
    CD_DEBUG("\n");
    return setControlModeRaw(joints[0], modes[0]);
}

// -----------------------------------------------------------------------------
