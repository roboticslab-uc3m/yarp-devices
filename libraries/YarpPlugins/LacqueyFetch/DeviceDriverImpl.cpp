// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "LacqueyFetch.hpp"

// -----------------------------------------------------------------------------
bool roboticslab::LacqueyFetch::open(yarp::os::Searchable& config)
{

    this->canId = config.check("canId",0,"can bus ID").asInt();
    this->tr = config.check("tr",0,"reduction").asInt();
    this->ptModeMs  = config.check("ptModeMs",0,"ptMode ms").asInt();
    this->ptPointCounter = 0;
    this->ptMovementDone = false;
    this->targetReached = false;
    this->max = 0;
    this->min = 0;
    this->refAcceleration = 0;
    this->refSpeed = 0;
    this->encoder = 0;

    yarp::os::Value vCanBufferFactory = config.check("canBufferFactory", 0, "");

    if( !vCanBufferFactory.isBlob() || vCanBufferFactory.asBlobLength() != sizeof(yarp::dev::ICanBufferFactory) )
    {
        CD_ERROR("Could not create LacqueyFetch with null or corrupt ICanBufferFactory handle\n");
        return false;
    }

    iCanBufferFactory = reinterpret_cast<yarp::dev::ICanBufferFactory *>(const_cast<char *>(vCanBufferFactory.asBlob()));
    canOutputBuffer = iCanBufferFactory->createBuffer(1);

    CD_SUCCESS("Created LacqueyFetch with canId %d and tr %f, and all local parameters set to 0.\n",canId,tr);
    return true;
}

// -----------------------------------------------------------------------------
bool roboticslab::LacqueyFetch::close()
{
    CD_INFO("\n");
    iCanBufferFactory->destroyBuffer(canOutputBuffer);
    return true;
}

// -----------------------------------------------------------------------------

