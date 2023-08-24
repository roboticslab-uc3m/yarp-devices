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
    canId = config.check("canId", yarp::os::Value(0), "can bus ID").asInt32();

    if (canId == 0)
    {
        yCError(DEXTRA) << "Could not create device with canId 0";
        return false;
    }

    yarp::dev::DeviceDriver::setId("ID" + std::to_string(canId));

    acquireSynapseHandle(new CanSynapse(canId));

    return DextraRawControlboard::open(config); // parses axisPrefix
}

// -----------------------------------------------------------------------------

bool DextraCanControlboard::close()
{
    destroySynapse();
    return true;
}

// -----------------------------------------------------------------------------
