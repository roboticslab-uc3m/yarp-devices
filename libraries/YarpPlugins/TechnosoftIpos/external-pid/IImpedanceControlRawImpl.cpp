// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "external-pid/TechnosoftIposExternal.hpp"

#include <yarp/os/LogStream.h>

#include "LogComponent.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue TechnosoftIposExternal::getImpedanceRaw(int j, double * stiffness, double * damping)
#else
bool TechnosoftIposExternal::getImpedanceRaw(int j, double * stiffness, double * damping)
#endif
{
    CHECK_JOINT(j);

    std::lock_guard lock(pidMutex);
    *stiffness = impedancePid.kp;
    *damping = impedancePid.kd;

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue::return_code::return_value_ok;
#else
    return true;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue TechnosoftIposExternal::setImpedanceRaw(int j, double stiffness, double damping)
#else
bool TechnosoftIposExternal::setImpedanceRaw(int j, double stiffness, double damping)
#endif
{
    CHECK_JOINT(j);

    if (stiffness < params.m_minStiffness || stiffness > params.m_maxStiffness)
    {
        yCIError(IPOS, id(), "Invalid stiffness: %f (not in [%f, %f])", stiffness, params.m_minStiffness, params.m_maxStiffness);
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
        return yarp::dev::ReturnValue::return_code::return_value_error_input_out_of_bounds;
#else
        return false;
#endif
    }

    if (damping < params.m_minDamping || damping > params.m_maxDamping)
    {
        yCIError(IPOS, id(), "Invalid damping: %f (not in [%f, %f])", damping, params.m_minDamping, params.m_maxDamping);
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
        return yarp::dev::ReturnValue::return_code::return_value_error_input_out_of_bounds;
#else
        return false;
#endif
    }

    std::lock_guard lock(pidMutex);
    impedancePid.setKp(stiffness);
    impedancePid.setKd(damping);
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue::return_code::return_value_ok;
#else
    return true;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue TechnosoftIposExternal::setImpedanceOffsetRaw(int j, double offset)
#else
bool TechnosoftIposExternal::setImpedanceOffsetRaw(int j, double offset)
#endif
{
    CHECK_JOINT(j);

    std::lock_guard lock(pidMutex);
    impedancePid.setOffset(offset);

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue::return_code::return_value_ok;
#else
    return true;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue TechnosoftIposExternal::getImpedanceOffsetRaw(int j, double * offset)
#else
bool TechnosoftIposExternal::getImpedanceOffsetRaw(int j, double * offset)
#endif
{
    CHECK_JOINT(j);

    std::lock_guard lock(pidMutex);
    *offset = impedancePid.offset;

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue::return_code::return_value_ok;
#else
    return true;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue TechnosoftIposExternal::getCurrentImpedanceLimitRaw(int j, double * min_stiff, double * max_stiff, double * min_damp, double * max_damp)
#else
bool TechnosoftIposExternal::getCurrentImpedanceLimitRaw(int j, double * min_stiff, double * max_stiff, double * min_damp, double * max_damp)
#endif
{
    CHECK_JOINT(j);

    *min_stiff = params.m_minStiffness;
    *max_stiff = params.m_maxStiffness;
    *min_damp = params.m_minDamping;
    *max_damp = params.m_maxDamping;

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue::return_code::return_value_ok;
#else
    return true;
#endif
}

// -----------------------------------------------------------------------------
