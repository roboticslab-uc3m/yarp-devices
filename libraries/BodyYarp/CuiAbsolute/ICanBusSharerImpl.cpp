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

    //-- Reserve space to copy data from CAN message that contains absolute encoder value
    float got;

    //-- Copy data from CAN message that contains absolute encoder value into "got"
    memcpy(&got, message->data,4);

    //CD_SUCCESS("Got absolute encoder value, as a float: %f\n",got);

    encoderReady.wait();

    //-- "got" is (0,360)degree,, move to (-180,180)degree range
    if (got > 180.0)
        got -= 360.0;

    //-- final used value (encoder) will be this value scaled by tr (usually just 1, or -1 to flip)
    encoder = got * this->tr;  //-- Even flipping, (-180,180)degree range will become (180,-180)degree range which is safe

    encoderTimestamp = message->ts;

    encoderReady.post();

    return true;

}  //-- ends interpretMessage

// -----------------------------------------------------------------------------
