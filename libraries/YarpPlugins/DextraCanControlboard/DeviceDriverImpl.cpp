// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "DextraCanControlboard.hpp"

#include <yarp/os/Value.h>

#include <ColorDebug.h>

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool DextraCanControlboard::open(yarp::os::Searchable & config)
{
    canId = config.check("canId", yarp::os::Value(0), "can bus ID").asInt32();

    if (canId == 0)
    {
        CD_ERROR("Could not create device with canId 0.\n");
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
