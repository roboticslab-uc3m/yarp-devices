// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "SpaceNavigator.hpp"

#include <cmath> // std::abs

#include <algorithm> // std::clamp, std::copysign

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
    if (std::abs(in) <= deadband)
    {
        return 0.0;
    }
    else
    {
        const double slope = RANGE / (RANGE - deadband);
        return slope * std::copysign(std::abs(in) - deadband, in);
    }
}
