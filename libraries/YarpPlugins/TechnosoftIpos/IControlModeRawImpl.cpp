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

    double refInternalUnits = vars.lastEncoderRead.queryPosition();
    double refDegrees = vars.internalUnitsToDegrees(refInternalUnits);

    if (!can->rpdo3()->configure(rpdo3Conf)
        || !can->sdo()->download<std::int8_t>("Modes of Operation", 7, 0x6060)
        || !can->sdo()->download<std::int16_t>("Interpolation sub mode select", linInterpBuffer->getSubMode(), 0x60C0)
        || !can->sdo()->download<std::uint16_t>("Interpolated position buffer length", linInterpBuffer->getBufferSize(), 0x2073)
        || !can->sdo()->download<std::uint16_t>("Interpolated position buffer configuration", 0xA000, 0x2074)
        || !can->sdo()->download<std::int32_t>("Interpolated position initial position", refInternalUnits, 0x2079))
    {
        return false;
    }

    linInterpBuffer->setInitialReference(refDegrees);
    linInterpBuffer->updateTarget(refDegrees);

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
    //CD_DEBUG("(%d)\n", j); // too verbose in controlboardwrapper2 stream
    CHECK_JOINT(j);
    *mode = vars.actualControlMode;
    return true;
}

// -----------------------------------------------------------------------------

bool TechnosoftIpos::getControlModesRaw(int * modes)
{
    //CD_DEBUG("\n"); // too verbose in controlboardwrapper2 stream
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

    if (mode == vars.actualControlMode
        || (mode == VOCAB_CM_CURRENT && vars.actualControlMode == VOCAB_CM_TORQUE)
        || (mode == VOCAB_CM_TORQUE && vars.actualControlMode == VOCAB_CM_CURRENT))
    {
        vars.actualControlMode.store(vars.requestedcontrolMode.load()); // disambiguate torque/current modes
        return true;
    }

    if (vars.actualControlMode == VOCAB_CM_POSITION_DIRECT
        && !can->driveStatus()->controlword(can->driveStatus()->controlword().reset(4))) // disable ip mode
    {
        return false;
    }

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
                && can->rpdo1()->write<std::uint16_t>(0x001F) // new setpoint (assume target position)
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
