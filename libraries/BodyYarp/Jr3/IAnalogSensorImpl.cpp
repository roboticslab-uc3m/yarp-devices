// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "Jr3.hpp"

// -----------------------------------------------------------------------------

int teo::Jr3::read(yarp::sig::Vector &out)
{
    int ret;
    ret = ioctl(fd,IOCTL0_JR3_FILTER0,&fm0);
    ret = ioctl(fd,IOCTL1_JR3_FILTER0,&fm1);
    ret = ioctl(fd,IOCTL2_JR3_FILTER0,&fm2);
    ret = ioctl(fd,IOCTL3_JR3_FILTER0,&fm3);

    if ( ret == -1)
    {
        CD_ERROR("\n");
        return yarp::dev::IAnalogSensor::AS_ERROR;
    }

    out.resize(DEFAULT_NUM_CHANNELS);

    out[0] = 100*fm0.f[0]*fs0.f[0]/16384;
    out[1] = 100*fm0.f[1]*fs0.f[1]/16384;
    out[2] = 100*fm0.f[2]*fs0.f[2]/16384;

    out[3] = fs0.m[0] / 16384.0;
    out[4] = fs0.m[1] / 16384.0;
    out[5] = fs0.m[2] / 16384.0;

    out[6] = fs1.f[0] / 16384.0;
    out[7] = fs1.f[1] / 16384.0;
    out[8] = fs1.f[2] / 16384.0;

    out[9] = fs1.m[0] / 16384.0;
    out[10] = fs1.m[1] / 16384.0;
    out[11] = fs1.m[2] / 16384.0;

    out[12] = fs2.f[0] / 16384.0;
    out[13] = fs2.f[1] / 16384.0;
    out[14] = fs2.f[2] / 16384.0;

    out[15] = fs2.m[0] / 16384.0;
    out[16] = fs2.m[1] / 16384.0;
    out[17] = fs2.m[2] / 16384.0;

    out[18] = fs3.f[0] / 16384.0;
    out[19] = fs3.f[1] / 16384.0;
    out[20] = fs3.f[2] / 16384.0;

    out[21] = fs3.m[0] / 16384.0;
    out[22] = fs3.m[1] / 16384.0;
    out[23] = fs3.m[2] / 16384.0;

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
