// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CuiAbsolute.hpp"

#include <yarp/os/Time.h>

#include <ColorDebug.h>

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool CuiAbsolute::getAxes(int * ax)
{
    *ax = 1;
    return true;
}

// -----------------------------------------------------------------------------

bool CuiAbsolute::resetEncoderRaw(int j)
{
    CD_DEBUG("(%d)\n", j);
    CHECK_JOINT(j);
    return setEncoderRaw(j, 0.0);
}

// -----------------------------------------------------------------------------

bool CuiAbsolute::resetEncodersRaw()
{
    CD_DEBUG("\n");
    return resetEncoderRaw(0);
}

// -----------------------------------------------------------------------------

bool CuiAbsolute::setEncoderRaw(int j, double val)
{
    CD_DEBUG("(%d, %f)\n", j, val);
    CHECK_JOINT(j);
    CD_WARNING("Not supported.\n");
    return false;
}

// -----------------------------------------------------------------------------

bool CuiAbsolute::setEncodersRaw(const double * vals)
{
    CD_WARNING("Not supported.\n");
    return false;
}

// -----------------------------------------------------------------------------

bool CuiAbsolute::getEncoderRaw(int j, double * v)
{
    //CD_DEBUG("%d\n",j); //-- Too verbose in stream.
    CHECK_JOINT(j);

    if (cuiMode == CuiMode::PULL)
    {
        return pollEncoderRead(v);
    }

    std::lock_guard<std::mutex> lock(mutex);
    *v = encoder;
    return true;
}

// -----------------------------------------------------------------------------

bool CuiAbsolute::getEncodersRaw(double * encs)
{
    CD_DEBUG("\n");
    return getEncoderRaw(0, &encs[0]);
}

// -----------------------------------------------------------------------------

bool CuiAbsolute::getEncoderSpeedRaw(int j, double * sp)
{
    //CD_DEBUG("(%d)\n",j); //-- Too verbose in controlboardwrapper2 stream.
    CHECK_JOINT(j);
    CD_WARNING("Not supported.\n");
    return false;
}

// -----------------------------------------------------------------------------

bool CuiAbsolute::getEncoderSpeedsRaw(double * spds)
{
    CD_WARNING("Not supported.\n");
    return false;
}

// -----------------------------------------------------------------------------

bool CuiAbsolute::getEncoderAccelerationRaw(int j, double * spds)
{
    //CD_DEBUG("(%d)\n",j); //-- Too verbose in controlboardwrapper2 stream.
    CHECK_JOINT(j);
    CD_WARNING("Not supported.\n");
    return false;
}

// -----------------------------------------------------------------------------

bool CuiAbsolute::getEncoderAccelerationsRaw(double * accs)
{
    CD_WARNING("Not supported.\n");
    return false;
}

// -----------------------------------------------------------------------------

bool CuiAbsolute::getEncodersTimedRaw(double * encs, double * times)
{
    CD_DEBUG("\n");
    return getEncoderTimedRaw(0, &encs[0], &times[0]);
}

// -----------------------------------------------------------------------------

bool CuiAbsolute::getEncoderTimedRaw(int j, double * enc, double * time)
{
    //CD_DEBUG("(%d)\n",j); //-- Too verbose in controlboardwrapper2 stream.
    CHECK_JOINT(j);

    if (cuiMode == CuiMode::PULL)
    {
        if (pollEncoderRead(enc))
        {
            *time = yarp::os::Time::now();
            return true;
        }
        else
        {
            return false;
        }
    }

    std::lock_guard<std::mutex> lock(mutex);
    *enc = encoder;
    *time = encoderTimestamp;
    return true;
}

// -----------------------------------------------------------------------------
