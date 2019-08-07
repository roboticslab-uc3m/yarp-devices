// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "DextraRawControlboard.hpp"

#include <algorithm>

#include <yarp/os/LockGuard.h>

using namespace roboticslab;

// -----------------------------------------------------------------------------

DextraRawControlboard::DextraRawControlboard()
    : synapse(0)
{}

// -----------------------------------------------------------------------------

void DextraRawControlboard::acquireSynapseHandle(Synapse * _synapse)
{
    synapse = _synapse;
}

// -----------------------------------------------------------------------------

void DextraRawControlboard::destroySynapse()
{
    if (synapse)
    {
        delete synapse;
        synapse = 0;
    }
}

// -----------------------------------------------------------------------------

double DextraRawControlboard::getSetpoint(int j)
{
    yarp::os::LockGuard lock(setpointMutex);
    return setpoints[j];
}

// -----------------------------------------------------------------------------

void DextraRawControlboard::getSetpoints(Synapse::Setpoints & setpoints)
{
    yarp::os::LockGuard lock(setpointMutex);
    std::copy(this->setpoints, this->setpoints + Synapse::DATA_POINTS, setpoints);
}

// -----------------------------------------------------------------------------

void DextraRawControlboard::setSetpoint(int j, Synapse::setpoint_t setpoint)
{
    yarp::os::LockGuard lock(setpointMutex);
    setpoints[j] = setpoint;
}

// -----------------------------------------------------------------------------

void DextraRawControlboard::setSetpoints(const Synapse::Setpoints & setpoints)
{
    yarp::os::LockGuard lock(setpointMutex);
    std::copy(setpoints, setpoints + Synapse::DATA_POINTS, this->setpoints);
}

// -----------------------------------------------------------------------------
