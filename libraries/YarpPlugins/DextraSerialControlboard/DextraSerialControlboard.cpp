// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "DextraSerialControlboard.hpp"

#include <algorithm>

using namespace roboticslab;

// -----------------------------------------------------------------------------

DextraSerialControlboard::DextraSerialControlboard()
    : synapse(0)
{}

// -----------------------------------------------------------------------------

double DextraSerialControlboard::getSetpoint(int j)
{
    std::lock_guard<std::mutex> lock(setpointMutex);
    return setpoints[j];
}

// -----------------------------------------------------------------------------

void DextraSerialControlboard::getSetpoints(Synapse::Setpoints & setpoints)
{
    std::lock_guard<std::mutex> lock(setpointMutex);
    std::copy(this->setpoints, this->setpoints + Synapse::DATA_POINTS, setpoints);
}

// -----------------------------------------------------------------------------

void DextraSerialControlboard::setSetpoint(int j, Synapse::setpoint_t setpoint)
{
    std::lock_guard<std::mutex> lock(setpointMutex);
    setpoints[j] = setpoint;
}

// -----------------------------------------------------------------------------

void DextraSerialControlboard::setSetpoints(const Synapse::Setpoints & setpoints)
{
    std::lock_guard<std::mutex> lock(setpointMutex);
    std::copy(setpoints, setpoints + Synapse::DATA_POINTS, this->setpoints);
}

// -----------------------------------------------------------------------------
