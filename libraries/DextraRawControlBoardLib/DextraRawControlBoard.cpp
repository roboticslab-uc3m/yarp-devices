// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "DextraRawControlBoard.hpp"

#include <algorithm>

using namespace roboticslab;

// -----------------------------------------------------------------------------

void DextraRawControlBoard::acquireSynapseHandle(Synapse * _synapse)
{
    synapse = _synapse;
}

// -----------------------------------------------------------------------------

void DextraRawControlBoard::destroySynapse()
{
    if (synapse)
    {
        delete synapse;
        synapse = nullptr;
    }
}

// -----------------------------------------------------------------------------

double DextraRawControlBoard::getSetpoint(int j)
{
    std::lock_guard lock(setpointMutex);
    return setpoints[j];
}

// -----------------------------------------------------------------------------

void DextraRawControlBoard::getSetpoints(Synapse::Setpoints & setpoints)
{
    std::lock_guard lock(setpointMutex);
    std::copy(std::cbegin(this->setpoints), std::cend(this->setpoints), std::begin(setpoints));
}

// -----------------------------------------------------------------------------

void DextraRawControlBoard::setSetpoint(int j, Synapse::setpoint_t setpoint)
{
    std::lock_guard lock(setpointMutex);
    setpoints[j] = setpoint;
}

// -----------------------------------------------------------------------------

void DextraRawControlBoard::setSetpoints(const Synapse::Setpoints & setpoints)
{
    std::lock_guard lock(setpointMutex);
    std::copy(std::cbegin(setpoints), std::cend(setpoints), std::begin(this->setpoints));
}

// -----------------------------------------------------------------------------
