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

    //CD_DEBUG("Got absolute encoder value. %s\n",msgToStr(message).c_str());
    float got;
    memcpy(&got, message->data,4);
    //CD_SUCCESS("Got absolute encoder value, as a float: %f\n",got);

    encoderReady.wait();

    encoder = got * this->tr;

    if (got < -180.0)
        got += 360.0;

    if (got > 180.0)
        got -= 360.0;

    encoderTimestamp = message->ts;

    encoderReady.post();

    return true;

}  //-- ends interpretMessage

// -----------------------------------------------------------------------------
