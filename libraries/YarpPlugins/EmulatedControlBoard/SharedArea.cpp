// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "EmulatedControlBoard.hpp"

using namespace roboticslab;

// ----------------------------------------------------------------------------

void EmulatedControlBoard::setEncRaw(const int index, const double position)
{
    encRawMutex.wait();
    encRaw[index] = position;
    encRawMutex.post();
}

// ----------------------------------------------------------------------------

void EmulatedControlBoard::setEncsRaw(const std::vector<double> & positions)
{
    encRawMutex.wait();
    encRaw = positions;
    encRawMutex.post();
}

// ----------------------------------------------------------------------------

double EmulatedControlBoard::getEncRaw(const int index)
{
    double position;
    encRawMutex.wait();
    position = encRaw[index];
    encRawMutex.post();
    return position;
}

// ----------------------------------------------------------------------------

std::vector<double> EmulatedControlBoard::getEncsRaw()
{
    std::vector<double> positions;
    encRawMutex.wait();
    positions = encRaw;
    encRawMutex.post();
    return positions;
}

// ----------------------------------------------------------------------------

double EmulatedControlBoard::getEncExposed(const int index)
{
    double rawPosition = getEncRaw(index);
    return rawPosition / encRawExposed[index];
}

// ----------------------------------------------------------------------------

std::vector<double> EmulatedControlBoard::getEncsExposed()
{
    std::vector<double> rawPositions = getEncsRaw();

    for (unsigned int i = 0; i < rawPositions.size(); i++)
    {
        rawPositions[i] /= encRawExposed[i];
    }

    return rawPositions;
}

// ----------------------------------------------------------------------------
