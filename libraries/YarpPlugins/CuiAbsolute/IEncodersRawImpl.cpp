// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CuiAbsolute.hpp"

#include <yarp/os/Log.h>
#include <yarp/os/Time.h>

#include "LogComponent.hpp"

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
    yCTrace(CUI, "%d", j);
    CHECK_JOINT(j);
    return setEncoderRaw(j, 0.0);
}

// -----------------------------------------------------------------------------

bool CuiAbsolute::resetEncodersRaw()
{
    return resetEncoderRaw(0);
}

// -----------------------------------------------------------------------------

bool CuiAbsolute::setEncoderRaw(int j, double val)
{
    yCTrace(CUI, "%d %f", j, val);
    CHECK_JOINT(j);
    yCWarning(CUI, "setEncoderRaw() not supported");
    return false;
}

// -----------------------------------------------------------------------------

bool CuiAbsolute::setEncodersRaw(const double * vals)
{
    yCWarning(CUI, "setEncodersRaw() not supported");
    return false;
}

// -----------------------------------------------------------------------------

bool CuiAbsolute::getEncoderRaw(int j, double * v)
{
    yCTrace(CUI, "%d", j);
    CHECK_JOINT(j);

    if (cuiMode == CuiMode::PULL)
    {
        encoder_t enc;

        if (!pollEncoderRead(&enc))
        {
            return false;
        }

        *v = enc;
        return true;
    }

    std::lock_guard<std::mutex> lock(mutex);
    *v = encoder;
    return true;
}

// -----------------------------------------------------------------------------

bool CuiAbsolute::getEncodersRaw(double * encs)
{
    return getEncoderRaw(0, &encs[0]);
}

// -----------------------------------------------------------------------------

bool CuiAbsolute::getEncoderSpeedRaw(int j, double * sp)
{
    yCTrace(CUI, "%d", j);
    CHECK_JOINT(j);
    yCWarning(CUI, "getEncoderSpeedRaw() not supported");
    return false;
}

// -----------------------------------------------------------------------------

bool CuiAbsolute::getEncoderSpeedsRaw(double * spds)
{
    yCWarning(CUI, "getEncoderSpeedsRaw() not supported");
    return false;
}

// -----------------------------------------------------------------------------

bool CuiAbsolute::getEncoderAccelerationRaw(int j, double * spds)
{
    yCTrace(CUI, "%d", j);
    CHECK_JOINT(j);
    yCWarning(CUI, "getEncoderAccelerationRaw() not supported");
    return false;
}

// -----------------------------------------------------------------------------

bool CuiAbsolute::getEncoderAccelerationsRaw(double * accs)
{
    yCWarning(CUI, "getEncoderAccelerationsRaw() not supported");
    return false;
}

// -----------------------------------------------------------------------------

bool CuiAbsolute::getEncodersTimedRaw(double * encs, double * times)
{
    return getEncoderTimedRaw(0, &encs[0], &times[0]);
}

// -----------------------------------------------------------------------------

bool CuiAbsolute::getEncoderTimedRaw(int j, double * enc, double * time)
{
    yCTrace(CUI, "%d", j);
    CHECK_JOINT(j);

    if (cuiMode == CuiMode::PULL)
    {
        encoder_t v;

        if (pollEncoderRead(&v))
        {
            *enc = v;
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
