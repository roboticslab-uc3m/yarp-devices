// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "DextraCanControlboard.hpp"

#include <yarp/os/Value.h>

#include <ColorDebug.h>

// -----------------------------------------------------------------------------

bool roboticslab::DextraCanControlboard::open(yarp::os::Searchable& config)
{
    /*
    canId = config.check("canId", yarp::os::Value(0), "can bus ID").asInt32();

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

    iCanBufferFactory = *reinterpret_cast<yarp::dev::ICanBufferFactory **>(const_cast<char *>(vCanBufferFactory.asBlob()));
    canOutputBuffer = iCanBufferFactory->createBuffer(1);
    */

    acquireSynapseHandle(new CanSynapse);

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::DextraCanControlboard::close()
{
    destroySynapse();
    //iCanBufferFactory->destroyBuffer(canOutputBuffer);
    return true;
}

// -----------------------------------------------------------------------------
