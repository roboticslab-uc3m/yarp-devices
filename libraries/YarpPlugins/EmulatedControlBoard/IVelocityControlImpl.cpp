// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "EmulatedControlBoard.hpp"

#include <yarp/os/LogStream.h>

#include "LogComponent.hpp"

// ------------------ IVelocity Related ----------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue EmulatedControlBoard::velocityMove(int j, double sp)
#else
bool EmulatedControlBoard::velocityMove(int j, double sp)
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

    if (controlMode != VELOCITY_MODE)
    {
        yCError(ECB) << "will not velocityMove as not in velocityMode";
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
        return yarp::dev::ReturnValue::return_code::return_value_error_not_ready;
#else
        return false;
#endif
    }

    velRaw[j] = sp * m_velRawExposeds[j];
    jointStatus[j] = VELOCITY_MOVE;

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue::return_code::return_value_ok;
#else
    return true;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue EmulatedControlBoard::velocityMove(const double * sp)
#else
bool EmulatedControlBoard::velocityMove(const double * sp)
#endif
{
    bool ok = true;

    for (unsigned int i = 0; i < m_axes; i++)
    {
        ok &= velocityMove(i, sp[i]);
    }

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return ok
        ? yarp::dev::ReturnValue::return_code::return_value_ok
        : yarp::dev::ReturnValue::return_code::return_value_error_method_failed;
#else
    return ok;
#endif
}

// ----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue EmulatedControlBoard::velocityMove(const int n_joint, const int * joints, const double * spds)
#else
bool EmulatedControlBoard::velocityMove(const int n_joint, const int * joints, const double * spds)
#endif
{
    bool ok = true;

    for (int i = 0; i < n_joint; i++)
    {
        ok &= velocityMove(joints[i], spds[i]);
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
yarp::dev::ReturnValue EmulatedControlBoard::getTargetVelocity(const int joint, double * vel)
#else
bool EmulatedControlBoard::getRefVelocity(const int joint, double * vel)
#endif
{
    yCWarning(ECB) << "getRefVelocity() not implemented yet";
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue::return_code::return_value_error_not_implemented_by_device;
#else
    return false;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue EmulatedControlBoard::getTargetVelocities(double * vels)
#else
bool EmulatedControlBoard::getRefVelocities(double * vels)
#endif
{
    yCWarning(ECB) << "getRefVelocities() not implemented yet";
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue::return_code::return_value_error_not_implemented_by_device;
#else
    return false;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue EmulatedControlBoard::getTargetVelocities(const int n_joint, const int * joints, double * vels)
#else
bool EmulatedControlBoard::getRefVelocities(const int n_joint, const int * joints, double * vels)
#endif
{
    yCWarning(ECB) << "getRefVelocities() not implemented yet";
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue::return_code::return_value_error_not_implemented_by_device;
#else
    return false;
#endif
}

// -----------------------------------------------------------------------------
