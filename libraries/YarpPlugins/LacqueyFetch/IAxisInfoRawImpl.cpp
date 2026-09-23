// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "LacqueyFetch.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue LacqueyFetch::getAxes(std::size_t & ax)
{
    int num;
    auto ret = getNumberOfMotorsRaw(&num);
    ax = static_cast<std::size_t>(num);
    return ret;
}
#else
bool LacqueyFetch::getAxes(int * ax)
{
    return getNumberOfMotorsRaw(ax);
}
#endif

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue LacqueyFetch::getAxisNameRaw(int axis, std::string & name)
#else
bool LacqueyFetch::getAxisNameRaw(int axis, std::string & name)
#endif
{
    CHECK_JOINT(axis);
    name = m_name;
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue::return_code::return_value_ok;
#else
    return true;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue LacqueyFetch::getJointTypeRaw(int axis, yarp::dev::JointTypeEnum & type)
#else
bool LacqueyFetch::getJointTypeRaw(int axis, yarp::dev::JointTypeEnum & type)
#endif
{
    CHECK_JOINT(axis);
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    type = yarp::dev::JointTypeEnum::VOCAB_JOINTTYPE_REVOLUTE;
    return yarp::dev::ReturnValue::return_code::return_value_ok;
#else
    type = yarp::dev::VOCAB_JOINTTYPE_REVOLUTE;
    return true;
#endif
}

// -----------------------------------------------------------------------------
