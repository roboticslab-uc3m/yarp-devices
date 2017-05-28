// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "AmorControlboard.hpp"

#define _USE_MATH_DEFINES
#include <cmath>

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

double roboticslab::AmorControlboard::toDeg(double rad)
{
    return rad * 180 / M_PI;
}

// -----------------------------------------------------------------------------

double roboticslab::AmorControlboard::toRad(double deg)
{
    return deg * M_PI / 180;
}

// -----------------------------------------------------------------------------
