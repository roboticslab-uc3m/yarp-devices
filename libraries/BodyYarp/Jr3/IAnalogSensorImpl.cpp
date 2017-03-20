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

    out[3] = 10*fm0.m[0]*fs0.m[0]/16384;
    out[4] = 10*fm0.m[1]*fs0.m[0]/16384;
    out[5] = 10*fm0.m[2]*fs0.m[0]/16384;

    out[6] = 100*fm1.f[0]*fs1.f[0]/16384;
    out[7] = 100*fm1.f[1]*fs1.f[1]/16384;
    out[8] = 100*fm1.f[2]*fs1.f[2]/16384;

    out[9] = 10*fm1.m[0]*fs1.m[0]/16384;
    out[10] = 10*fm1.m[1]*fs1.m[0]/16384;
    out[11] = 10*fm1.m[2]*fs1.m[0]/16384;

    out[12] = 100*fm2.f[0]*fs2.f[0]/16384;
    out[13] = 100*fm2.f[1]*fs2.f[1]/16384;
    out[14] = 100*fm2.f[2]*fs2.f[2]/16384;

    out[15] = 10*fm2.m[0]*fs2.m[0]/16384;
    out[16] = 10*fm2.m[1]*fs2.m[0]/16384;
    out[17] = 10*fm2.m[2]*fs2.m[0]/16384;

    out[18] = 100*fm3.f[0]*fs3.f[0]/16384;
    out[19] = 100*fm3.f[1]*fs3.f[1]/16384;
    out[20] = 100*fm3.f[2]*fs3.f[2]/16384;

    out[21] = 10*fm3.m[0]*fs3.m[0]/16384;
    out[22] = 10*fm3.m[1]*fs3.m[0]/16384;
    out[23] = 10*fm3.m[2]*fs3.m[0]/16384;

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
