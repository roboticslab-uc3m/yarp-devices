// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "EmulatedControlBoard.hpp"

// ----------------------------------------------------------------------------

void EmulatedControlBoard::setEncRaw(const int index, const double position)
{
    encRawMutex.lock();
    encRaw[index] = position;
    encRawMutex.unlock();
}

// ----------------------------------------------------------------------------

void EmulatedControlBoard::setEncsRaw(const std::vector<double> & positions)
{
    encRawMutex.lock();
    encRaw = positions;
    encRawMutex.unlock();
}

// ----------------------------------------------------------------------------

double EmulatedControlBoard::getEncRaw(const int index)
{
    double position;
    encRawMutex.lock();
    position = encRaw[index];
    encRawMutex.unlock();
    return position;
}

// ----------------------------------------------------------------------------

std::vector<double> EmulatedControlBoard::getEncsRaw()
{
    std::vector<double> positions;
    encRawMutex.lock();
    positions = encRaw;
    encRawMutex.unlock();
    return positions;
}

// ----------------------------------------------------------------------------

double EmulatedControlBoard::getEncExposed(const int index)
{
    double rawPosition = getEncRaw(index);
    return rawPosition / m_encRawExposeds[index];
}

// ----------------------------------------------------------------------------

std::vector<double> EmulatedControlBoard::getEncsExposed()
{
    std::vector<double> rawPositions = getEncsRaw();

    for (unsigned int i = 0; i < rawPositions.size(); i++)
    {
        rawPositions[i] /= m_encRawExposeds[i];
    }

    return rawPositions;
}

// ----------------------------------------------------------------------------
