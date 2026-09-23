// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TechnosoftIposBase.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue TechnosoftIposBase::getLastJointFaultRaw(int axis, int & fault, std::string & message)
#else
bool TechnosoftIposBase::getLastJointFaultRaw(int axis, int & fault, std::string & message)
#endif
{
    CHECK_JOINT(axis);
    fault = lastFaultCode;
    message = lastFaultMessage;
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue::return_code::return_value_ok;
#else
    return true;
#endif
}

// -----------------------------------------------------------------------------
