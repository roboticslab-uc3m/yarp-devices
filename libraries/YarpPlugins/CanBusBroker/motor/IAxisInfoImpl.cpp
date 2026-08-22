// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusBroker.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue CanBusBroker::getAxisName(int axis, std::string & name)
#else
bool CanBusBroker::getAxisName(int axis, std::string & name)
#endif
{
    CHECK_JOINT(axis);
    auto fn = &yarp::dev::IAxisInfoRaw::getAxisNameRaw;
    return deviceMapper.mapSingleJoint<yarp::dev::IAxisInfoRaw, std::string &>(fn, axis, name);
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue CanBusBroker::getJointType(int axis, yarp::dev::JointTypeEnum & type)
#else
bool CanBusBroker::getJointType(int axis, yarp::dev::JointTypeEnum & type)
#endif
{
    CHECK_JOINT(axis);
    auto fn = &yarp::dev::IAxisInfoRaw::getJointTypeRaw;
    return deviceMapper.mapSingleJoint<yarp::dev::IAxisInfoRaw, yarp::dev::JointTypeEnum &>(fn, axis, type);
}

// -----------------------------------------------------------------------------
