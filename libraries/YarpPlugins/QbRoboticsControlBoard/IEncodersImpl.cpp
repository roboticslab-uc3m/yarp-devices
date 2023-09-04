// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "QbRoboticsControlBoard.hpp"

using namespace roboticslab;

// ------------------ IEncoders Related -----------------------------------------

bool QbRoboticsControlBoard::resetEncoder(int j)
{
    return false;
}

// -----------------------------------------------------------------------------

bool QbRoboticsControlBoard::resetEncoders()
{
    return false;
}

// -----------------------------------------------------------------------------

bool QbRoboticsControlBoard::setEncoder(int j, double val)  // encExposed = val;
{
    return false;
}

// -----------------------------------------------------------------------------

bool QbRoboticsControlBoard::setEncoders(const double * vals)
{
    return false;
}

// -----------------------------------------------------------------------------

bool QbRoboticsControlBoard::getEncoder(int j, double * v)
{
    return false;
}

// -----------------------------------------------------------------------------

bool QbRoboticsControlBoard::getEncoders(double * encs)
{
    return false;
}

// -----------------------------------------------------------------------------

bool QbRoboticsControlBoard::getEncoderSpeed(int j, double * sp)
{
    return false;
}

// -----------------------------------------------------------------------------

bool QbRoboticsControlBoard::getEncoderSpeeds(double * spds)
{
    return false;
}

// -----------------------------------------------------------------------------

bool QbRoboticsControlBoard::getEncoderAcceleration(int j, double * spds)
{
    return false;
}

// -----------------------------------------------------------------------------

bool QbRoboticsControlBoard::getEncoderAccelerations(double * accs)
{
    return false;
}

// ------------------ IEncodersTimed Related -----------------------------------------

bool QbRoboticsControlBoard::getEncodersTimed(double * encs, double * time)
{
    return false;
}

// -----------------------------------------------------------------------------

bool QbRoboticsControlBoard::getEncoderTimed(int j, double * encs, double * time)
{
    return false;
}

// -----------------------------------------------------------------------------
