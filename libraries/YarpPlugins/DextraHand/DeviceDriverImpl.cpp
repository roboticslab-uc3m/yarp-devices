// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "DextraHand.hpp"

// -----------------------------------------------------------------------------
bool roboticslab::DextraHand::open(yarp::os::Searchable& config)
{

    /*this->canId = config.check("canId",0,"can bus ID").asInt();
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

    CD_SUCCESS("Created DextraHand with canId %d and tr %f, and all local parameters set to 0.\n",canId,tr);
    */

    char serialport[13] = "/dev/ttyUSB0";
    int baudrate = B115200;  // Should match https://github.com/Alvipe/Dextra/blob/master/Control/DextraControl.py
    char buf[256];
    int rc,n;

    fd = serialport_init(serialport, baudrate);

    if ( fd <= 0 )
    {
        CD_ERROR("fd = %d <= 0, bye!\n",fd);
        return false;
    }

    CD_SUCCESS("open(), fd: %d\n",fd);

    return true;
}

// -----------------------------------------------------------------------------
bool roboticslab::DextraHand::close()
{
    CD_INFO("\n");
    return true;
}

// -----------------------------------------------------------------------------

