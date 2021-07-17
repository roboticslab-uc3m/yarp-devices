// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "DextraCanControlboard.hpp"

#include <yarp/os/LogComponent.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Value.h>

using namespace roboticslab;

namespace
{
    YARP_LOG_COMPONENT(DEXTRA, "rl.DextraCanControlboard")
}

// -----------------------------------------------------------------------------

bool DextraCanControlboard::open(yarp::os::Searchable & config)
{
    yCDebug(DEXTRA) << "Config:" << config.toString();

    canId = config.check("canId", yarp::os::Value(0), "can bus ID").asInt32();

    if (canId == 0)
    {
        yCError(DEXTRA) << "Could not create device with canId 0";
        return false;
    }

    acquireSynapseHandle(new CanSynapse(canId));

    return true;
}

// -----------------------------------------------------------------------------

bool DextraCanControlboard::close()
{
    destroySynapse();
    return true;
}

// -----------------------------------------------------------------------------
