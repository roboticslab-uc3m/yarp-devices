// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TextilesHand.hpp"

// -----------------------------------------------------------------------------
bool teo::TextilesHand::open(yarp::os::Searchable& config) {

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

    CD_SUCCESS("Created TextilesHand with canId %d and tr %f, and all local parameters set to 0.\n",canId,tr);
    */

    char serialport[13] = "/dev/ttyUSB0";
    int baudrate = B9600;  // default
    char buf[256];
    int rc,n;

    fd = serialport_init(serialport, baudrate);
    if(!fd) {
        printf("NULL fd, bye!\n");
        ::exit(-1);
    }
    CD_SUCCESS("open(), fd: %d\n",fd);

    return true;
}

// -----------------------------------------------------------------------------
bool teo::TextilesHand::close() {
    CD_INFO("\n");
    return true;
}

// -----------------------------------------------------------------------------

