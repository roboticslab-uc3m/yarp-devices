// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "AmorControlboard.hpp"

// ------------------- IControlMode2 related ------------------------------------

bool roboticslab::AmorControlboard::getControlModes(const int n_joint, const int *joints, int *modes)
{
    CD_DEBUG("(%d)\n", n_joint);

    if (!batchWithinRange(n_joint))
    {
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::setControlMode(const int j, const int mode)
{
    CD_DEBUG("(%d, %d)\n", j, mode);

    if (!indexWithinRange(j))
    {
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::setControlModes(const int n_joint, const int *joints, int *modes)
{
    CD_DEBUG("(%d)\n", n_joint);

    if (!batchWithinRange(n_joint))
    {
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::setControlModes(int *modes)
{
    CD_DEBUG("\n");
    return true;
}

// -----------------------------------------------------------------------------
