// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "SpaceNavigator.hpp"

double roboticslab::SpaceNavigator::enforceRange(double in)
{
    double out;

    if (in > 1)
    {
        out = 1;
    }
    else
    {
        if (in < -1)
        {
            out = -1;
        }
        else
        {
            out = in;
        }
    }
    return out;
}

// -----------------------------------------------------------------------------

double roboticslab::SpaceNavigator::enforceDeadband(double in)
{
    double out;

    if (in > DEADBAND)
    {
        out = in;
    }
    else
    {
        if (in < -DEADBAND)
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
