// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "EmulatedControlboard.hpp"

using namespace roboticslab;

// ----------------------------------------------------------------------------

void EmulatedControlboard::setEncRaw(const int index, const double position)
{
    encRawMutex.wait();
    encRaw[index] = position;
    encRawMutex.post();
}

// ----------------------------------------------------------------------------

void EmulatedControlboard::setEncsRaw(const std::vector<double> & positions)
{
    encRawMutex.wait();
    encRaw = positions;
    encRawMutex.post();
}

// ----------------------------------------------------------------------------

double EmulatedControlboard::getEncRaw(const int index)
{
    double position;
    encRawMutex.wait();
    position = encRaw[index];
    encRawMutex.post();
    return position;
}

// ----------------------------------------------------------------------------

std::vector<double> EmulatedControlboard::getEncsRaw()
{
    std::vector<double> positions;
    encRawMutex.wait();
    positions = encRaw;
    encRawMutex.post();
    return positions;
}

// ----------------------------------------------------------------------------

double EmulatedControlboard::getEncExposed(const int index)
{
    double rawPosition = getEncRaw(index);
    return rawPosition / encRawExposed[index];
}

// ----------------------------------------------------------------------------

std::vector<double> EmulatedControlboard::getEncsExposed()
{
    std::vector<double> rawPositions = getEncsRaw();

    for (unsigned int i = 0; i < rawPositions.size(); i++)
    {
        rawPositions[i] /= encRawExposed[i];
    }

    return rawPositions;
}

// ----------------------------------------------------------------------------
