// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "QbRoboticsControlBoard.hpp"

#include <yarp/os/Log.h>

#include "LogComponent.hpp"

using namespace roboticslab;

// ------------------ IVelocity Related ----------------------------------------

bool QbRoboticsControlBoard::velocityMove(int j, double sp)
{
    yCWarning(QBROB, "velocityMove() not implemented yet");
    return false;
}

// -----------------------------------------------------------------------------

bool QbRoboticsControlBoard::velocityMove(const double *sp)
{
    yCWarning(QBROB, "velocityMove() not implemented yet");
    return false;
}

// ----------------------------------------------------------------------------

bool QbRoboticsControlBoard::velocityMove(int n_joint, const int * joints, const double * spds)
{
    yCWarning(QBROB, "velocityMove() not implemented yet");
    return false;
}

// -----------------------------------------------------------------------------

bool QbRoboticsControlBoard::getRefVelocity(int joint, double * vel)
{
    yCWarning(QBROB, "getRefVelocity() not implemented yet");
    return false;
}

// -----------------------------------------------------------------------------

bool QbRoboticsControlBoard::getRefVelocities(double * vels)
{
    yCWarning(QBROB, "getRefVelocities() not implemented yet");
    return false;
}

// -----------------------------------------------------------------------------

bool QbRoboticsControlBoard::getRefVelocities(int n_joint, const int * joints, double * vels)
{
    yCWarning(QBROB, "getRefVelocities() not implemented yet");
    return false;
}

// -----------------------------------------------------------------------------
