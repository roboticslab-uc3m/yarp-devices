// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CuiAbsolute.hpp"

// -----------------------------------------------------------------------------
bool roboticslab::CuiAbsolute::open(yarp::os::Searchable& config)
{

    this->canId = config.check("canId",yarp::os::Value(0),"can bus ID").asInt();
    this->tr = config.check("tr",yarp::os::Value(1),"reduction").asInt();
    this->ptModeMs  = config.check("ptModeMs",yarp::os::Value(0),"ptMode (milliseconds)").asInt();
    this->ptPointCounter = 0;
    this->ptMovementDone = false;
    this->targetReached = false;
    this->max = 0;
    this->min = 0;
    this->refAcceleration = 0;
    this->refSpeed = 0;
    this->encoder = sqrt (-1);  // NaN \todo{Investigate, debug and document the dangers of this use of NaN.}

    yarp::os::Value vCanBufferFactory = config.check("canBufferFactory", 0, "");

    if( !vCanBufferFactory.isBlob() || vCanBufferFactory.asBlobLength() != sizeof(yarp::dev::ICanBufferFactory) )
    {
        CD_ERROR("Could not create CuiAbsolute with null or corrupt ICanBufferFactory handle\n");
        return false;
    }

    iCanBufferFactory = reinterpret_cast<yarp::dev::ICanBufferFactory *>(const_cast<char *>(vCanBufferFactory.asBlob()));
    canOutputBuffer = iCanBufferFactory->createBuffer(1);

    CD_SUCCESS("Created CuiAbsolute with canId %d and tr %f, and all local parameters set to 0.\n",canId,tr);
    return true;
}

// -----------------------------------------------------------------------------
bool roboticslab::CuiAbsolute::close()
{
    CD_INFO("\n");
    iCanBufferFactory->destroyBuffer(canOutputBuffer);
    return true;
}

// -----------------------------------------------------------------------------

