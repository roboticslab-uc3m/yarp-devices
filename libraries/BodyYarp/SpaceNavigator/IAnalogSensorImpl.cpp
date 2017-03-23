// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "SpaceNavigator.hpp"

// -----------------------------------------------------------------------------

int roboticslab::SpaceNavigator::read(yarp::sig::Vector &out)
{
    return yarp::dev::IAnalogSensor::AS_OK;
}

// -----------------------------------------------------------------------------

int roboticslab::SpaceNavigator::getState(int ch)
{
    return yarp::dev::IAnalogSensor::AS_OK;
}

// -----------------------------------------------------------------------------

int roboticslab::SpaceNavigator::getChannels()
{
    return DEFAULT_NUM_CHANNELS;
}

// -----------------------------------------------------------------------------

int roboticslab::SpaceNavigator::calibrateSensor()
{
    return true;
}

// -----------------------------------------------------------------------------

int roboticslab::SpaceNavigator::calibrateSensor(const yarp::sig::Vector& value)
{
    return true;
}

// -----------------------------------------------------------------------------

int roboticslab::SpaceNavigator::calibrateChannel(int ch)
{
    return true;
}

// -----------------------------------------------------------------------------

int roboticslab::SpaceNavigator::calibrateChannel(int ch, double value)
{
    return true;
}

// -----------------------------------------------------------------------------
