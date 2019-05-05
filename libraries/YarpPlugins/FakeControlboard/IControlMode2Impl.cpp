// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "FakeControlboard.hpp"

#include <ColorDebug.h>

// ------------------- IControlMode2 Related ------------------------------------

bool roboticslab::FakeControlboard::getControlModes(const int n_joint, const int *joints, int *modes)
{
    CD_DEBUG("(%d)\n", n_joint);
    bool ok = true;

    for (int i = 0; i < n_joint; i++)
    {
        ok &= getControlMode(joints[i], &modes[i]);
    }

    return ok;
}

// -----------------------------------------------------------------------------

bool roboticslab::FakeControlboard::setControlMode(const int j, const int mode)
{
    CD_DEBUG("(%d)\n", j);

    if ((unsigned int)j > axes)
    {
        CD_ERROR("axis index more than axes.\n");
        return false;
    }

    switch (mode)
    {
    case VOCAB_CM_POSITION:
        return setPositionMode(j);
    case VOCAB_CM_VELOCITY:
        return setVelocityMode(j);
    case VOCAB_CM_TORQUE:
        return setTorqueMode(j);
    case VOCAB_CM_POSITION_DIRECT:
        return setPositionDirectMode(j);
    default:
        return false;
    }
}

// -----------------------------------------------------------------------------

bool roboticslab::FakeControlboard::setControlModes(const int n_joint, const int *joints, int *modes)
{
    CD_DEBUG("(%d)\n", n_joint);
    bool ok = true;

    for (int j = 0; j < n_joint; j++)
    {
        ok &= setControlMode(joints[j], modes[j]);
    }

    return ok;
}

// -----------------------------------------------------------------------------

bool roboticslab::FakeControlboard::setControlModes(int *modes)
{
    CD_DEBUG("\n");
    bool ok = true;

    for (unsigned int i = 0; i < axes; i++)
    {
        ok &= setControlMode(i, modes[i]);
    }

    return ok;
}

// -----------------------------------------------------------------------------
