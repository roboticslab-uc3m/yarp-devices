// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TechnosoftIpos.hpp"

#include <yarp/conf/version.h>
#include <yarp/os/Log.h>

#include "LogComponent.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool TechnosoftIpos::getLastJointFaultRaw(int axis, int & fault, std::string & message)
{
#if defined(YARP_VERSION_COMPARE) // >= 3.6.0
    yCITrace(IPOS, id(), "%d", axis);
#else
    yCTrace(IPOS, "%d", axis);
#endif
    CHECK_JOINT(axis);
    fault = vars.lastFaultCode;
    message = vars.lastFaultMessage;
    return true;
}

// -----------------------------------------------------------------------------
