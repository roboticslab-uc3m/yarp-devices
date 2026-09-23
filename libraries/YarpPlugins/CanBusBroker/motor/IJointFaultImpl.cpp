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
    return deviceMapper.mapSingleJoint(&yarp::dev::IJointFaultRaw::getLastJointFaultRaw, axis, fault, message);
}

// -----------------------------------------------------------------------------
