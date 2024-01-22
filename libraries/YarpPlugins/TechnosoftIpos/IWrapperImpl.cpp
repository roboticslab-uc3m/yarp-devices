// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TechnosoftIposBase.hpp"

#include <algorithm> // std::find_if
#include <string>

#include <yarp/os/LogStream.h>

#include "LogComponent.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool TechnosoftIposBase::attach(yarp::dev::PolyDriver * driver)
{
    if (auto it = std::find_if(disabledEncoderIds.begin(), disabledEncoderIds.end(),
                               [driver](int id) { return "ID" + std::to_string(id) == driver->id(); });
        it != disabledEncoderIds.end())
    {
        yCIInfo(IPOS, id()) << "Skipping attach of disabled external encoder" << driver->id();
        return true;
    }

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

    return true;
}

// -----------------------------------------------------------------------------

bool TechnosoftIposBase::detach()
{
    return true;
}

// -----------------------------------------------------------------------------
