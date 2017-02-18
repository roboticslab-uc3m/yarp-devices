// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "Jr3.hpp"

// -----------------------------------------------------------------------------

int teo::Jr3::read(yarp::sig::Vector &out)
{
    int ret = ioctl(fd,IOCTL0_JR3_FILTER0,&fm);

    if ( ret == -1)
    {
        return yarp::dev::IAnalogSensor::AS_ERROR;
    }

    out.resize(DEFAULT_NUM_CHANNELS);

    out[0] = 100.0*(fm.f[0])*(fs.f[0])/16384.0;
    out[1] = 100.0*(fm.f[1])*(fs.f[1])/16384.0;
    out[2] = 100.0*(fm.f[2])*(fs.f[2])/16384.0;

    out[3] = 10.0*(fm.m[0])*(fs.m[0])/16384.0;
    out[4] = 10.0*(fm.m[1])*(fs.m[1])/16384.0;
    out[5] = 10.0*(fm.m[2])*(fs.m[2])/16384.0;

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
