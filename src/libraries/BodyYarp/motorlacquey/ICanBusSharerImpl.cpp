// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "MotorLacquey.hpp"

// -----------------------------------------------------------------------------

bool teo::MotorLacquey::setCanBusPtr(CanBusHico *canDevicePtr) {

    this->canDevicePtr = canDevicePtr;
    CD_SUCCESS("Ok pointer to CAN bus device %d.\n",canId);

}

// -----------------------------------------------------------------------------

bool teo::MotorLacquey::start() {

    return true;
}

// -----------------------------------------------------------------------------

bool teo::MotorLacquey::readyToSwitchOn() {

    return true;
}

// -----------------------------------------------------------------------------

bool teo::MotorLacquey::switchOn() {

    return true;
}

// -----------------------------------------------------------------------------

bool teo::MotorLacquey::enable() {

    return true;
}

// -----------------------------------------------------------------------------

bool teo::MotorLacquey::recoverFromError() {

    return true;
}

// -----------------------------------------------------------------------------

bool teo::MotorLacquey::interpretMessage( can_msg * message) {

    return true;

}  //-- ends interpretMessage

// -----------------------------------------------------------------------------
