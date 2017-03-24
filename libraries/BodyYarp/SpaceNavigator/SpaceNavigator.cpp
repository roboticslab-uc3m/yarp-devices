// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "SpaceNavigator.hpp"

const unsigned int roboticslab::SpaceNavigator::MAX_NO_DATA_ITERATIONS = 10;

// -----------------------------------------------------------------------------

roboticslab::SpaceNavigator::SpaceNavigator()
    : dx(0.0), dy(0.0), dz(0.0),
      droll(0.0), dpitch(0.0), dyaw(0.0),
      button1(0), button2(0),
      noDataCounter(0)
{
}

// -----------------------------------------------------------------------------

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
