// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "SpaceNavigator.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

const unsigned int roboticslab::SpaceNavigator::MAX_NO_DATA_ITERATIONS = 10;

// -----------------------------------------------------------------------------

SpaceNavigator::SpaceNavigator()
    : dx(0.0), dy(0.0), dz(0.0),
      droll(0.0), dpitch(0.0), dyaw(0.0),
      button1(0), button2(0),
      noDataCounter(0), deadband(0.0)
{ }

// -----------------------------------------------------------------------------

SpaceNavigator::~SpaceNavigator()
{
    close();
}

// -----------------------------------------------------------------------------

double SpaceNavigator::enforceRange(double in)
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
