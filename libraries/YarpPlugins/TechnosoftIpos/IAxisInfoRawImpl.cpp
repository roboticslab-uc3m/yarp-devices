// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TechnosoftIposBase.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue TechnosoftIposBase::getAxisNameRaw(int axis, std::string & name)
#else
bool TechnosoftIposBase::getAxisNameRaw(int axis, std::string & name)
#endif
{
    CHECK_JOINT(axis);
    name = params.m_name;
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue_ok;
#else
    return true;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue TechnosoftIposBase::getJointTypeRaw(int axis, yarp::dev::JointTypeEnum & type)
#else
bool TechnosoftIposBase::getJointTypeRaw(int axis, yarp::dev::JointTypeEnum & type)
#endif
{
    CHECK_JOINT(axis);
    type = static_cast<yarp::dev::JointTypeEnum>(jointType);
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue_ok;
#else
    return true;
#endif
}

// -----------------------------------------------------------------------------
