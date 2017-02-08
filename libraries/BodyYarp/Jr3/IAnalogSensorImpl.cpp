// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "Jr3.hpp"

// -----------------------------------------------------------------------------

int teo::Jr3::read(yarp::sig::Vector &out)
{
    fmSemaphore.wait();

    out.resize(DEFAULT_NUM_CHANNELS);

    out[0] = f[0];
    out[1] = f[1];
    out[2] = f[2];

    out[3] = m[0];
    out[4] = m[1];
    out[5] = m[2];

    fmSemaphore.post();

    return yarp::dev::IAnalogSensor::AS_OK;
}

// -----------------------------------------------------------------------------

int teo::Jr3::getState(int ch)
{
    return yarp::dev::IAnalogSensor::AS_OK;
}

// -----------------------------------------------------------------------------

int teo::Jr3::getChannels()
{
    return DEFAULT_NUM_CHANNELS;
}

// -----------------------------------------------------------------------------

int teo::Jr3::calibrateSensor()
{
    return true;
}

// -----------------------------------------------------------------------------

int teo::Jr3::calibrateSensor(const yarp::sig::Vector& value)
{
    return true;
}

// -----------------------------------------------------------------------------

int teo::Jr3::calibrateChannel(int ch)
{
    return true;
}

// -----------------------------------------------------------------------------

int teo::Jr3::calibrateChannel(int ch, double value)
{
    return true;
}

// -----------------------------------------------------------------------------
