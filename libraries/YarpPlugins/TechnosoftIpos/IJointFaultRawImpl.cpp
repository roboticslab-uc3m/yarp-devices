// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TechnosoftIposBase.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool TechnosoftIposBase::getLastJointFaultRaw(int axis, int & fault, std::string & message)
{
    CHECK_JOINT(axis);
    fault = lastFaultCode;
    message = lastFaultMessage;
    return true;
}

// -----------------------------------------------------------------------------
