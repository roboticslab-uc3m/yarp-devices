// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CuiAbsolute.hpp"

// -----------------------------------------------------------------------------

bool teo::CuiAbsolute::setCanBusPtr(CanBusHico *canDevicePtr) {

    this->canDevicePtr = canDevicePtr;
    CD_SUCCESS("Ok pointer to CAN bus device %d.\n",canId);

}

// -----------------------------------------------------------------------------

bool teo::CuiAbsolute::start() {

    return true;
}

// -----------------------------------------------------------------------------

bool teo::CuiAbsolute::readyToSwitchOn() {

    return true;
}

// -----------------------------------------------------------------------------

bool teo::CuiAbsolute::switchOn() {

    return true;
}

// -----------------------------------------------------------------------------

bool teo::CuiAbsolute::enable() {

    return true;
}

// -----------------------------------------------------------------------------

bool teo::CuiAbsolute::recoverFromError() {

    return true;
}

// -----------------------------------------------------------------------------

bool teo::CuiAbsolute::interpretMessage( can_msg * message) {

    CD_DEBUG("Got absolute encoder value. %s\n",msgToStr(message).c_str());
    int gotd;
    memcpy(&gotd, message->data+4,4);
    CD_SUCCESS("Got absolute encoder value, as an int: %d\n",gotd);
    float gotf;
    memcpy(&gotf, message->data+4,4);
    CD_SUCCESS("Got absolute encoder value, as a float: %f\n",gotf);

    return true;

}  //-- ends interpretMessage

// -----------------------------------------------------------------------------
