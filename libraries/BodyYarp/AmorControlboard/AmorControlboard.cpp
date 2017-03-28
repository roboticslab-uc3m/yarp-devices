// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "AmorControlboard.hpp"

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::indexWithinRange(const int& idx)
{
    if (idx >= AMOR_NUM_JOINTS)
    {
        CD_WARNING("Index out of range!! (%d >= %d)!!!\n", idx, AMOR_NUM_JOINTS);
        return false;
    }
    return true;
}

// -----------------------------------------------------------------------------
