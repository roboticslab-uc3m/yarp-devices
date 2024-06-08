// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "SpaceNavigator.hpp"

#include <algorithm> // std::clamp

using namespace roboticslab;

constexpr auto RANGE = 1.0;

// -----------------------------------------------------------------------------

double SpaceNavigator::enforceRange(double in)
{
    return std::clamp(in, -RANGE, RANGE);
}

// -----------------------------------------------------------------------------

double SpaceNavigator::enforceDeadband(double in)
{
    double out;

    if (in > deadband)
    {
        out = in;
    }
    else
    {
        if (in < -deadband)
        {
            out = in;
        }
        else
        {
            out = 0.0;
        }
    }
    return out;
}
