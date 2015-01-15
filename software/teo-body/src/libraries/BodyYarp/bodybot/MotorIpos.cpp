// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "MotorDriver.hpp"

// -----------------------------------------------------------------------------

MotorDriver::MotorDriver(CanBusHico *canDevicePtr, const int& canId, const double &tr) : max(0), min(0), refAcceleration(0), refSpeed(0), encoder(0) {
    this->canDevicePtr = canDevicePtr;
    this->canId = canId;
    this->tr = tr;
    this->ptPointCounter = 0;
    this->ptMovementDone = false;
    this->targetReached = false;
    CD_SUCCESS("Created driver with canId %d and tr %f, pointer to CAN bus device, and all local parameters set to 0.\n",canId,tr);
}

// -----------------------------------------------------------------------------

int MotorDriver::getCanId() {
    return canId;
}

// -----------------------------------------------------------------------------

double MotorDriver::getEncoder() {
    double value;
    encoderReady.wait();
    value = encoder;
    encoderReady.post();
    return value;
}

// -----------------------------------------------------------------------------

double MotorDriver::getMax() {
    return max;
}

// -----------------------------------------------------------------------------

double MotorDriver::getMin() {
    return min;
}

// -----------------------------------------------------------------------------

double MotorDriver::getRefAcceleration() {
    return refAcceleration;
}

// -----------------------------------------------------------------------------

double MotorDriver::getRefSpeed() {
    return refSpeed;
}

// -----------------------------------------------------------------------------

double MotorDriver::getTr() {
    return tr;
}

// -----------------------------------------------------------------------------

void MotorDriver::setEncoder(const double& value) {
    encoderReady.wait();
    encoder = value;
    encoderReady.post();
}

// -----------------------------------------------------------------------------

void MotorDriver::setMax(const double& value) {
    max = value;
}

// ----------------------------.-------------------------------------------------

void MotorDriver::setMin(const double& value) {
    min = value;
}

// -----------------------------------------------------------------------------

void MotorDriver::setRefAcceleration(const double& value) {
    refAcceleration = value;
}

// -----------------------------------------------------------------------------

void MotorDriver::setRefSpeed(const double& value) {
    refSpeed = value;
}

// -----------------------------------------------------------------------------

void MotorDriver::setTr(const double& value) {
    tr = value;
}

// -----------------------------------------------------------------------------

bool MotorDriver::send(uint32_t cob, uint16_t len, uint8_t * msgData) {

    if ( (lastUsage - yarp::os::Time::now()) < DELAY )
        yarp::os::Time::delay( lastUsage + DELAY - yarp::os::Time::now() );

    if( ! canDevicePtr->sendRaw(cob + this->canId, len, msgData) )
        return false;

    lastUsage = yarp::os::Time::now();
    return true;
}

// -----------------------------------------------------------------------------

bool MotorDriver::start() {

    //*************************************************************
    uint8_t msg_start[] = {0x01,0x01};

    msg_start[1]=this->getCanId();
    if( ! canDevicePtr->sendRaw(0, 2, msg_start) )
    {
        CD_ERROR("Could not send \"start\" to canId: %d.\n",this->getCanId());
        return false;
    }
    CD_SUCCESS("Sent \"start\" to canId: %d.\n",this->getCanId());
    //*************************************************************

    return true;
}

// -----------------------------------------------------------------------------

bool MotorDriver::readyToSwitchOn() {

    //*************************************************************
    uint8_t msg_readyToSwitchOn[] = {0x06,0x00}; //-- readyToSwitchOn, also acts as shutdown.

    if( ! this->send( 0x200, 2, msg_readyToSwitchOn) )
    {
        CD_ERROR("Could not send \"readyToSwitchOn/shutdown\" to canId: %d.\n",this->getCanId());
        return false;
    }
    CD_SUCCESS("Sent \"readyToSwitchOn/shutdown\" to canId: %d.\n",this->getCanId());
    //*************************************************************

    return true;
}

// -----------------------------------------------------------------------------

bool MotorDriver::switchOn() {

    //*************************************************************
    uint8_t msg_switchOn[] = {0x07,0x00};  //-- switchOn, also acts as disableOperation
    if( ! this->send( 0x200, 2, msg_switchOn) )
    {
        CD_ERROR("Could not send \"switchOn/disableOperation\" to canId: %d.\n",this->getCanId());
        return false;
    }
    CD_SUCCESS("Sent \"switchOn/disableOperation\"  to canId: %d. Should respond...\n",this->getCanId());
    //* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
    yarp::os::Time::delay(0.1);
    //* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
    struct can_msg replyOn;
    while(canDevicePtr->read_timeout(&replyOn,20))
    {
        CD_SUCCESS("Got response to \"switchOn/disableOperation\" from canId: %d.\n",replyOn.id & 0x7F);
    }
    //*************************************************************

    return true;
}

// -----------------------------------------------------------------------------

bool MotorDriver::enable() {

    //*************************************************************
    uint8_t msg_enable[] = {0x0F,0x00}; // enable

    if( ! this->send( 0x200, 2, msg_enable) )
    {
        CD_ERROR("Could not send \"enable\" to canId: %d.\n",this->getCanId());
        return false;
    }
    CD_SUCCESS("Sent \"enable\" to canId: %d. Should respond...\n",this->getCanId());
    //* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
    yarp::os::Time::delay(0.1);
    //* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
    struct can_msg reply;
    while(canDevicePtr->read_timeout(&reply,20))
    {
        CD_SUCCESS("Got response to \"enable\" from canId: %d.\n",reply.id & 0x7F);
    }
    //*************************************************************

    return true;
}

// -----------------------------------------------------------------------------
