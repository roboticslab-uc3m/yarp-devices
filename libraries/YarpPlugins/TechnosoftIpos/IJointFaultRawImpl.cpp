// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TechnosoftIposBase.hpp"

#include <yarp/os/Log.h>

#include "LogComponent.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool TechnosoftIposBase::getLastJointFaultRaw(int axis, int & fault, std::string & message)
{
    yCITrace(IPOS, id(), "%d", axis);
    CHECK_JOINT(axis);
    fault = vars.lastFaultCode;
    message = vars.lastFaultMessage;
    return true;
}

// -----------------------------------------------------------------------------
