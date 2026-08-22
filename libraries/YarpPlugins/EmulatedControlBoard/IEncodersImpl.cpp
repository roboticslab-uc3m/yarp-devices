// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "EmulatedControlBoard.hpp"

#include <yarp/os/LogStream.h>

#include "LogComponent.hpp"

// ------------------ IEncoders Related -----------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue EmulatedControlBoard::resetEncoder(int j)
#else
bool EmulatedControlBoard::resetEncoder(int j)
#endif
{
    if (j < 0 || j >= m_axes)
    {
        yCError(ECB) << "Axis index out of bounds: " << j;
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
        return yarp::dev::ReturnValue::return_code::return_value_error_input_out_of_bounds;
#else
        return false;
#endif
    }

    return setEncoder(j, 0.0);
  }

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue EmulatedControlBoard::resetEncoders()
#else
bool EmulatedControlBoard::resetEncoders()
#endif
{
    bool ok = true;

    for (unsigned int i = 0; i < m_axes; i++)
    {
        ok &= resetEncoder(i);
    }

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return ok
        ? yarp::dev::ReturnValue::return_code::return_value_ok
        : yarp::dev::ReturnValue::return_code::return_value_error_method_failed;
#else
    return ok;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue EmulatedControlBoard::setEncoder(int j, double val)
#else
bool EmulatedControlBoard::setEncoder(int j, double val)
#endif
{
    if (j < 0 || j >= m_axes)
    {
        yCError(ECB) << "Axis index out of bounds: " << j;
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
        return yarp::dev::ReturnValue::return_code::return_value_error_input_out_of_bounds;
#else
        return false;
#endif
    }

    setEncRaw(j, val * m_encRawExposeds[j]);
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue::return_code::return_value_ok;
#else
    return true;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue EmulatedControlBoard::setEncoders(const double * vals)
#else
bool EmulatedControlBoard::setEncoders(const double * vals)
#endif
{
    std::vector<double> v(m_axes);

    for (unsigned int i = 0; i < m_axes; i++)
    {
        v[i] = vals[i] * m_encRawExposeds[i];
    }

    setEncsRaw(v);
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue::return_code::return_value_ok;
#else
    return true;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue EmulatedControlBoard::getEncoder(int j, double * v)
#else
bool EmulatedControlBoard::getEncoder(int j, double * v)
#endif
{
    if (j < 0 || j >= m_axes)
    {
        yCError(ECB) << "Axis index out of bounds: " << j;
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
        return yarp::dev::ReturnValue::return_code::return_value_error_input_out_of_bounds;
#else
        return false;
#endif
    }

    *v = getEncExposed(j);
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue::return_code::return_value_ok;
#else
    return true;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue EmulatedControlBoard::getEncoders(double * encs)
#else
bool EmulatedControlBoard::getEncoders(double * encs)
#endif
{
    std::vector<double> v = getEncsExposed();

    for (unsigned int i = 0; i < m_axes; i++)
    {
        encs[i] = v[i];
    }

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue::return_code::return_value_ok;
#else
    return true;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue EmulatedControlBoard::getEncoderSpeed(int j, double * sp)
#else
bool EmulatedControlBoard::getEncoderSpeed(int j, double *sp)
#endif
{
    if (j < 0 || j >= m_axes)
    {
        yCError(ECB) << "Axis index out of bounds: " << j;
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
        return yarp::dev::ReturnValue::return_code::return_value_error_input_out_of_bounds;
#else
        return false;
#endif
    }

    // Make it easy, give the current reference speed.
    *sp = velRaw[j] / m_velRawExposeds[j];  // begins to look like we should use semaphores.
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue::return_code::return_value_ok;
#else
    return true;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue EmulatedControlBoard::getEncoderSpeeds(double * spds)
#else
bool EmulatedControlBoard::getEncoderSpeeds(double * spds)
#endif
{
    bool ok = true;

    for (unsigned int i = 0; i < m_axes; i++)
    {
        ok &= getEncoderSpeed(i, &spds[i]);
    }

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return ok
        ? yarp::dev::ReturnValue::return_code::return_value_ok
        : yarp::dev::ReturnValue::return_code::return_value_error_method_failed;
#else
    return ok;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue EmulatedControlBoard::getEncoderAcceleration(int j, double * spds)
{
    return yarp::dev::ReturnValue::return_code::return_value_error_not_implemented_by_device;
}
#else
bool EmulatedControlBoard::getEncoderAcceleration(int j, double * spds)
{
    return false;
}
#endif

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue EmulatedControlBoard::getEncoderAccelerations(double * accs)
{
    return yarp::dev::ReturnValue::return_code::return_value_error_not_implemented_by_device;
}
#else
bool EmulatedControlBoard::getEncoderAccelerations(double * accs)
{
    return false;
}
#endif

// ------------------ IEncodersTimed Related -----------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue EmulatedControlBoard::getEncodersTimed(double * encs, double * time)
#else
bool EmulatedControlBoard::getEncodersTimed(double * encs, double * time)
#endif
{
    bool ok = true;

    for (unsigned int i = 0; i < m_axes; i++)
    {
        ok &= getEncoderTimed(i, &encs[i], &time[i]);
    }

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return ok
        ? yarp::dev::ReturnValue::return_code::return_value_ok
        : yarp::dev::ReturnValue::return_code::return_value_error_method_failed;
#else
    return ok;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue EmulatedControlBoard::getEncoderTimed(int j, double * encs, double * time)
#else
bool EmulatedControlBoard::getEncoderTimed(int j, double * encs, double * time)
#endif
{
    auto ret = getEncoder(j, encs);
    *time = yarp::os::Time::now();

    return ret;
}

// -----------------------------------------------------------------------------
