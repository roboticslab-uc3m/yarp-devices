// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CuiAbsolute.hpp"

#include <yarp/conf/version.h>

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
#if YARP_VERSION_MINOR >= 6
    yCITrace(CUI, id(), "%d", j);
#else
    yCTrace(CUI, "%d", j);
#endif
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
#if YARP_VERSION_MINOR >= 6
    yCITrace(CUI, id(), "%d %f", j, val);
#else
    yCTrace(CUI, "%d %f", j, val);
#endif
    CHECK_JOINT(j);
#if YARP_VERSION_MINOR >= 6
    yCIWarning(CUI, id(), "setEncoderRaw() not supported");
#else
    yCWarning(CUI, "setEncoderRaw() not supported");
#endif
    return false;
}

// -----------------------------------------------------------------------------

bool CuiAbsolute::setEncodersRaw(const double * vals)
{
#if YARP_VERSION_MINOR >= 6
    yCIWarning(CUI, id(), "setEncodersRaw() not supported");
#else
    yCWarning(CUI, "setEncodersRaw() not supported");
#endif
    return false;
}

// -----------------------------------------------------------------------------

bool CuiAbsolute::getEncoderRaw(int j, double * v)
{
#if YARP_VERSION_MINOR >= 6
    yCITrace(CUI, id(), "%d", j);
#else
    yCTrace(CUI, "%d", j);
#endif
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
#if YARP_VERSION_MINOR >= 6
    yCITrace(CUI, id(), "%d", j);
#else
    yCTrace(CUI, "%d", j);
#endif
    CHECK_JOINT(j);
#if YARP_VERSION_MINOR >= 6
    yCIWarning(CUI, id(), "getEncoderSpeedRaw() not supported");
#else
    yCWarning(CUI, "getEncoderSpeedRaw() not supported");
#endif
    return false;
}

// -----------------------------------------------------------------------------

bool CuiAbsolute::getEncoderSpeedsRaw(double * spds)
{
#if YARP_VERSION_MINOR >= 6
    yCIWarning(CUI, id(), "getEncoderSpeedsRaw() not supported");
#else
    yCWarning(CUI, "getEncoderSpeedsRaw() not supported");
#endif
    return false;
}

// -----------------------------------------------------------------------------

bool CuiAbsolute::getEncoderAccelerationRaw(int j, double * spds)
{
#if YARP_VERSION_MINOR >= 6
    yCITrace(CUI, id(), "%d", j);
#else
    yCTrace(CUI, "%d", j);
#endif
    CHECK_JOINT(j);
#if YARP_VERSION_MINOR >= 6
    yCIWarning(CUI, id(), "getEncoderAccelerationRaw() not supported");
#else
    yCWarning(CUI, "getEncoderAccelerationRaw() not supported");
#endif
    return false;
}

// -----------------------------------------------------------------------------

bool CuiAbsolute::getEncoderAccelerationsRaw(double * accs)
{
#if YARP_VERSION_MINOR >= 6
    yCIWarning(CUI, id(), "getEncoderAccelerationsRaw() not supported");
#else
    yCWarning(CUI, "getEncoderAccelerationsRaw() not supported");
#endif
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
#if YARP_VERSION_MINOR >= 6
    yCITrace(CUI, id(), "%d", j);
#else
    yCTrace(CUI, "%d", j);
#endif
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
