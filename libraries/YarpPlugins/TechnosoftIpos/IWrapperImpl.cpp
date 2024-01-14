// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TechnosoftIposBase.hpp"

using namespace roboticslab;

#include <yarp/os/LogStream.h>

#include "LogComponent.hpp"

// -----------------------------------------------------------------------------

bool TechnosoftIposBase::attach(yarp::dev::PolyDriver * driver)
{
    if (!driver->view(iEncodersTimedRawExternal))
    {
        yCIError(IPOS, id()) << "Unable to view IEncodersTimedRaw in" << driver->id();
        return false;
    }

    if (!driver->view(iExternalEncoderCanBusSharer))
    {
        yCIError(IPOS, id()) << "Unable to view ICanBusSharer in" << driver->id();
        return false;
    }

    return false;
}

// -----------------------------------------------------------------------------

bool TechnosoftIposBase::detach()
{
    return true;
}

// -----------------------------------------------------------------------------
