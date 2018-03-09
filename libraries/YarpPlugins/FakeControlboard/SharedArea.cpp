// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "FakeControlboard.hpp"

// ----------------------------------------------------------------------------

void roboticslab::FakeControlboard::setEncRaw(const int index, const double position)
{
    encRawMutex.wait();
    encRaw[index] = position;
    encRawMutex.post();
}

// ----------------------------------------------------------------------------

void roboticslab::FakeControlboard::setEncsRaw(const std::vector<double> & positions)
{
    encRawMutex.wait();
    encRaw = positions;
    encRawMutex.post();
}

// ----------------------------------------------------------------------------

double roboticslab::FakeControlboard::getEncRaw(const int index)
{
    double position;
    encRawMutex.wait();
    position = encRaw[index];
    encRawMutex.post();
    return position;
}

// ----------------------------------------------------------------------------

std::vector<double> roboticslab::FakeControlboard::getEncsRaw()
{
    std::vector<double> positions;
    encRawMutex.wait();
    positions = encRaw;
    encRawMutex.post();
    return positions;
}

// ----------------------------------------------------------------------------

double roboticslab::FakeControlboard::getEncExposed(const int index)
{
    double rawPosition = getEncRaw(index);
    return rawPosition / encRawExposed[index];
}

// ----------------------------------------------------------------------------

std::vector<double> roboticslab::FakeControlboard::getEncsExposed()
{
    std::vector<double> rawPositions = getEncsRaw();

    for (unsigned int i = 0; i < rawPositions.size(); i++)
    {
        rawPositions[i] /= encRawExposed[i];
    }

    return rawPositions;
}

// ----------------------------------------------------------------------------
