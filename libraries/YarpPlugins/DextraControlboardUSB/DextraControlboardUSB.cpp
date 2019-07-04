// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "DextraControlboardUSB.hpp"

#include <yarp/os/LockGuard.h>

using namespace roboticslab;

// -----------------------------------------------------------------------------

double DextraControlboardUSB::getSetpoint(int j)
{
    yarp::os::LockGuard lock(setpointMutex);
    return setpoints[j];
}

// -----------------------------------------------------------------------------

std::vector<double> DextraControlboardUSB::getSetpoints()
{
    yarp::os::LockGuard lock(setpointMutex);
    return setpoints;
}

// -----------------------------------------------------------------------------

void DextraControlboardUSB::setSetpoint(int j, double setpoint)
{
    yarp::os::LockGuard lock(setpointMutex);
    setpoints[j] = setpoint;
}

// -----------------------------------------------------------------------------

void DextraControlboardUSB::setSetpoints(const std::vector<double> & setpoints)
{
    yarp::os::LockGuard lock(setpointMutex);
    this->setpoints = setpoints;
}

// -----------------------------------------------------------------------------
