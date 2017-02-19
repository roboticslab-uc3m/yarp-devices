// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "PlaybackThread.hpp"

// -----------------------------------------------------------------------------

int teo::PlaybackThread::read(yarp::sig::Vector &out)
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

int teo::PlaybackThread::getState(int ch)
{
    return yarp::dev::IAnalogSensor::AS_OK;
}

// -----------------------------------------------------------------------------

int teo::PlaybackThread::getChannels()
{
    return DEFAULT_NUM_CHANNELS;
}

// -----------------------------------------------------------------------------

int teo::PlaybackThread::calibrateSensor()
{
    return true;
}

// -----------------------------------------------------------------------------

int teo::PlaybackThread::calibrateSensor(const yarp::sig::Vector& value)
{
    return true;
}

// -----------------------------------------------------------------------------

int teo::PlaybackThread::calibrateChannel(int ch)
{
    return true;
}

// -----------------------------------------------------------------------------

int teo::PlaybackThread::calibrateChannel(int ch, double value)
{
    return true;
}

// -----------------------------------------------------------------------------
