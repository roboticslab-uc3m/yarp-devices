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
    yCITrace(CUI, id(), "%d", j);
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
    yCITrace(CUI, id(), "%d %f", j, val);
    CHECK_JOINT(j);
    yCIWarning(CUI, id(), "setEncoderRaw() not supported");
    return false;
}

// -----------------------------------------------------------------------------

bool CuiAbsolute::setEncodersRaw(const double * vals)
{
    yCIWarning(CUI, id(), "setEncodersRaw() not supported");
    return false;
}

// -----------------------------------------------------------------------------

bool CuiAbsolute::getEncoderRaw(int j, double * v)
{
    yCITrace(CUI, id(), "%d", j);
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
    yCITrace(CUI, id(), "%d", j);
    CHECK_JOINT(j);
    yCIWarning(CUI, id(), "getEncoderSpeedRaw() not supported");
    return false;
}

// -----------------------------------------------------------------------------

bool CuiAbsolute::getEncoderSpeedsRaw(double * spds)
{
    yCIWarning(CUI, id(), "getEncoderSpeedsRaw() not supported");
    return false;
}

// -----------------------------------------------------------------------------

bool CuiAbsolute::getEncoderAccelerationRaw(int j, double * spds)
{
    yCITrace(CUI, id(), "%d", j);
    CHECK_JOINT(j);
    yCIWarning(CUI, id(), "getEncoderAccelerationRaw() not supported");
    return false;
}

// -----------------------------------------------------------------------------

bool CuiAbsolute::getEncoderAccelerationsRaw(double * accs)
{
    yCIWarning(CUI, id(), "getEncoderAccelerationsRaw() not supported");
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
    yCITrace(CUI, id(), "%d", j);
    CHECK_JOINT(j);

    if (cuiMode == CuiMode::PULL)
    {
        if (encoder_t v; pollEncoderRead(&v))
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
