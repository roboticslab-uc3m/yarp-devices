// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CuiAbsolute.hpp"

#include <yarp/conf/version.h>

#include <yarp/os/LogStream.h>
#include <yarp/os/Time.h>

#include "LogComponent.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue CuiAbsolute::getAxes(std::size_t & ax)
{
    ax = 1;
    return yarp::dev::ReturnValue::return_code::return_value_ok;
}
#else
bool CuiAbsolute::getAxes(int * ax)
{
    *ax = 1;
    return true;
}
#endif

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue CuiAbsolute::resetEncoderRaw(int j)
#else
bool CuiAbsolute::resetEncoderRaw(int j)
#endif
{
    CHECK_JOINT(j);
    return setEncoderRaw(j, 0.0);
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue CuiAbsolute::resetEncodersRaw()
#else
bool CuiAbsolute::resetEncodersRaw()
#endif
{
    return resetEncoderRaw(0);
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue CuiAbsolute::setEncoderRaw(int j, double val)
#else
bool CuiAbsolute::setEncoderRaw(int j, double val)
#endif
{
    CHECK_JOINT(j);
    yCIWarning(CUI, id()) << "setEncoderRaw() not supported";
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue::return_code::return_value_error_not_implemented_by_device;
#else
    return false;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue CuiAbsolute::setEncodersRaw(const double * vals)
#else
bool CuiAbsolute::setEncodersRaw(const double * vals)
#endif
{
    yCIWarning(CUI, id()) << "setEncodersRaw() not supported";
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue::return_code::return_value_error_not_implemented_by_device;
#else
    return false;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue CuiAbsolute::getEncoderRaw(int j, double * v)
#else
bool CuiAbsolute::getEncoderRaw(int j, double * v)
#endif
{
    CHECK_JOINT(j);

    if (cuiMode == CuiMode::PULL)
    {
        encoder_t enc;

        if (!pollEncoderRead(&enc))
        {
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
            return yarp::dev::ReturnValue::return_code::return_value_error_method_failed;
#else
            return false;
#endif
        }

        *v = enc;
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
        return yarp::dev::ReturnValue::return_code::return_value_ok;
#else
        return true;
#endif
    }

    std::lock_guard lock(mutex);
    *v = encoder;
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue::return_code::return_value_ok;
#else
    return true;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue CuiAbsolute::getEncodersRaw(double * encs)
#else
bool CuiAbsolute::getEncodersRaw(double * encs)
#endif
{
    return getEncoderRaw(0, &encs[0]);
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue CuiAbsolute::getEncoderSpeedRaw(int j, double * sp)
#else
bool CuiAbsolute::getEncoderSpeedRaw(int j, double * sp)
#endif
{
    CHECK_JOINT(j);
    yCIWarning(CUI, id()) << "getEncoderSpeedRaw() not supported";
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue::return_code::return_value_error_not_implemented_by_device;
#else
    return false;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue CuiAbsolute::getEncoderSpeedsRaw(double * spds)
#else
bool CuiAbsolute::getEncoderSpeedsRaw(double * spds)
#endif
{
    yCIWarning(CUI, id()) << "getEncoderSpeedsRaw() not supported";
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue::return_code::return_value_error_not_implemented_by_device;
#else
    return false;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue CuiAbsolute::getEncoderAccelerationRaw(int j, double * spds)
#else
bool CuiAbsolute::getEncoderAccelerationRaw(int j, double * spds)
#endif
{
    CHECK_JOINT(j);
    yCIWarning(CUI, id()) << "getEncoderAccelerationRaw() not supported";
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue::return_code::return_value_error_not_implemented_by_device;
#else
    return false;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue CuiAbsolute::getEncoderAccelerationsRaw(double * accs)
#else
bool CuiAbsolute::getEncoderAccelerationsRaw(double * accs)
#endif
{
    yCIWarning(CUI, id()) << "getEncoderAccelerationsRaw() not supported";
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue::return_code::return_value_error_not_implemented_by_device;
#else
    return false;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue CuiAbsolute::getEncodersTimedRaw(double * encs, double * times)
#else
bool CuiAbsolute::getEncodersTimedRaw(double * encs, double * times)
#endif
{
    return getEncoderTimedRaw(0, &encs[0], &times[0]);
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue CuiAbsolute::getEncoderTimedRaw(int j, double * enc, double * time)
#else
bool CuiAbsolute::getEncoderTimedRaw(int j, double * enc, double * time)
#endif
{
    CHECK_JOINT(j);

    if (cuiMode == CuiMode::PULL)
    {
        if (encoder_t v; pollEncoderRead(&v))
        {
            *enc = v;
            *time = yarp::os::Time::now();
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
            return yarp::dev::ReturnValue::return_code::return_value_ok;
#else
            return true;
#endif
        }
        else
        {
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
            return yarp::dev::ReturnValue::return_code::return_value_error_method_failed;
#else
            return false;
#endif
        }
    }

    std::lock_guard lock(mutex);
    *enc = encoder;
    *time = encoderTimestamp;
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue::return_code::return_value_ok;
#else
    return true;
#endif
}

// -----------------------------------------------------------------------------
