// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusBroker.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue CanBusBroker::getLastJointFault(int axis, int & fault, std::string & message)
#else
bool CanBusBroker::getLastJointFault(int axis, int & fault, std::string & message)
#endif
{
    CHECK_JOINT(axis);
    auto fn = &yarp::dev::IJointFaultRaw::getLastJointFaultRaw;
    return deviceMapper.mapSingleJoint<yarp::dev::IJointFaultRaw, int &, std::string &>(fn, axis, fault, message);
}

// -----------------------------------------------------------------------------
