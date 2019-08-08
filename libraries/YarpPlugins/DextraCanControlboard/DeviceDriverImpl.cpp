// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "DextraCanControlboard.hpp"

#include <yarp/os/Value.h>

#include <ColorDebug.h>

// -----------------------------------------------------------------------------

bool roboticslab::DextraCanControlboard::open(yarp::os::Searchable& config)
{
    int canId = config.check("canId", yarp::os::Value(0), "can bus ID").asInt32();

    if (canId == 0)
    {
        CD_ERROR("Could not create device with canId 0.\n");
        return false;
    }

    yarp::os::Value vCanBufferFactory = config.check("canBufferFactory", yarp::os::Value(0), "");

    if (!vCanBufferFactory.isBlob())
    {
        CD_ERROR("Could not create device with null or corrupt ICanBufferFactory handle\n");
        return false;
    }

    yarp::dev::ICanBufferFactory * iCanBufferFactory =
            *reinterpret_cast<yarp::dev::ICanBufferFactory **>(const_cast<char *>(vCanBufferFactory.asBlob()));

    yarp::dev::ICanBus * canDevicePtr; // FIXME: empty canDevicePtr

    acquireSynapseHandle(new CanSynapse(canId, canDevicePtr, iCanBufferFactory));

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::DextraCanControlboard::close()
{
    destroySynapse();
    return true;
}

// -----------------------------------------------------------------------------
