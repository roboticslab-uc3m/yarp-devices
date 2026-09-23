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
    CHECK_JOINT(j);

    if (controlMode != VOCAB_CM_VELOCITY)
    {
        yCError(ECB) << "will not velocityMove as not in velocityMode";
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
        return yarp::dev::ReturnValue_error_not_ready;
#else
        return false;
#endif
    }

    velRaw[j] = sp * m_velRawExposeds[j];
    jointStatus[j] = VELOCITY_MOVE;

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue_ok;
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
    return ok ? yarp::dev::ReturnValue_ok : yarp::dev::ReturnValue_error_method_failed;
#else
    return ok;
#endif
}

// ----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue EmulatedControlBoard::velocityMove(int n_joint, const int * joints, const double * spds)
#else
bool EmulatedControlBoard::velocityMove(int n_joint, const int * joints, const double * spds)
#endif
{
    bool ok = true;

    for (int i = 0; i < n_joint; i++)
    {
        ok &= velocityMove(joints[i], spds[i]);
    }

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return ok ? yarp::dev::ReturnValue_ok : yarp::dev::ReturnValue_error_method_failed;
#else
    return ok;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue EmulatedControlBoard::getTargetVelocity(int joint, double * vel)
#else
bool EmulatedControlBoard::getRefVelocity(int joint, double * vel)
#endif
{
    yCWarning(ECB) << "getRefVelocity() not implemented yet";
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue_error_not_implemented_by_device;
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
    return yarp::dev::ReturnValue_error_not_implemented_by_device;
#else
    return false;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue EmulatedControlBoard::getTargetVelocities(int n_joint, const int * joints, double * vels)
#else
bool EmulatedControlBoard::getRefVelocities(int n_joint, const int * joints, double * vels)
#endif
{
    yCWarning(ECB) << "getRefVelocities() not implemented yet";
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue_error_not_implemented_by_device;
#else
    return false;
#endif
}

// -----------------------------------------------------------------------------
