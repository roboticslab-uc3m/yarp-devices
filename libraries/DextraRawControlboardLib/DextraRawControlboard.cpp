// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "DextraRawControlboard.hpp"

#include <algorithm>

using namespace roboticslab;

// -----------------------------------------------------------------------------

DextraRawControlboard::DextraRawControlboard()
    : synapse(nullptr)
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
        synapse = nullptr;
    }
}

// -----------------------------------------------------------------------------

double DextraRawControlboard::getSetpoint(int j)
{
    std::lock_guard<std::mutex> lock(setpointMutex);
    return setpoints[j];
}

// -----------------------------------------------------------------------------

void DextraRawControlboard::getSetpoints(Synapse::Setpoints & setpoints)
{
    std::lock_guard<std::mutex> lock(setpointMutex);
    std::copy(std::cbegin(this->setpoints), std::cend(this->setpoints), std::begin(setpoints));
}

// -----------------------------------------------------------------------------

void DextraRawControlboard::setSetpoint(int j, Synapse::setpoint_t setpoint)
{
    std::lock_guard<std::mutex> lock(setpointMutex);
    setpoints[j] = setpoint;
}

// -----------------------------------------------------------------------------

void DextraRawControlboard::setSetpoints(const Synapse::Setpoints & setpoints)
{
    std::lock_guard<std::mutex> lock(setpointMutex);
    std::copy(std::cbegin(setpoints), std::cend(setpoints), std::begin(this->setpoints));
}

// -----------------------------------------------------------------------------
