// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "DextraControlboardUSB.hpp"

#include <algorithm>

#include <yarp/os/LockGuard.h>

using namespace roboticslab;

// -----------------------------------------------------------------------------

double DextraControlboardUSB::getSetpoint(int j)
{
    yarp::os::LockGuard lock(setpointMutex);
    return setpoints[j];
}

// -----------------------------------------------------------------------------

void DextraControlboardUSB::getSetpoints(Synapse::Setpoints & setpoints)
{
    yarp::os::LockGuard lock(setpointMutex);
    std::copy(this->setpoints, this->setpoints + Synapse::DATA_POINTS, setpoints);
}

// -----------------------------------------------------------------------------

void DextraControlboardUSB::setSetpoint(int j, Synapse::setpoint_t setpoint)
{
    yarp::os::LockGuard lock(setpointMutex);
    setpoints[j] = setpoint;
}

// -----------------------------------------------------------------------------

void DextraControlboardUSB::setSetpoints(const Synapse::Setpoints & setpoints)
{
    yarp::os::LockGuard lock(setpointMutex);
    std::copy(setpoints, setpoints + Synapse::DATA_POINTS, this->setpoints);
}

// -----------------------------------------------------------------------------
