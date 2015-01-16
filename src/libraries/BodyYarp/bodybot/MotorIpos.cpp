// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "MotorIpos.hpp"

// -----------------------------------------------------------------------------

MotorIpos::MotorIpos(CanBusHico *canDevicePtr, const int& canId, const double &tr) : max(0), min(0), refAcceleration(0), refSpeed(0), encoder(0) {
    this->canDevicePtr = canDevicePtr;
    this->canId = canId;
    this->tr = tr;
    this->ptPointCounter = 0;
    this->ptMovementDone = false;
    this->targetReached = false;
    CD_SUCCESS("Created driver with canId %d and tr %f, pointer to CAN bus device, and all local parameters set to 0.\n",canId,tr);
}

// -----------------------------------------------------------------------------

int MotorIpos::getCanId() {
    return canId;
}

// -----------------------------------------------------------------------------

double MotorIpos::getEncoder() {
    double value;
    encoderReady.wait();
    value = encoder;
    encoderReady.post();
    return value;
}

// -----------------------------------------------------------------------------

double MotorIpos::getMax() {
    return max;
}

// -----------------------------------------------------------------------------

double MotorIpos::getMin() {
    return min;
}

// -----------------------------------------------------------------------------

double MotorIpos::getRefAcceleration() {
    return refAcceleration;
}

// -----------------------------------------------------------------------------

double MotorIpos::getRefSpeed() {
    return refSpeed;
}

// -----------------------------------------------------------------------------

double MotorIpos::getTr() {
    return tr;
}

// -----------------------------------------------------------------------------

void MotorIpos::setEncoder(const double& value) {
    encoderReady.wait();
    encoder = value;
    encoderReady.post();
}

// -----------------------------------------------------------------------------

void MotorIpos::setMax(const double& value) {
    max = value;
}

// ----------------------------.-------------------------------------------------

void MotorIpos::setMin(const double& value) {
    min = value;
}

// -----------------------------------------------------------------------------

void MotorIpos::setRefAcceleration(const double& value) {
    refAcceleration = value;
}

// -----------------------------------------------------------------------------

void MotorIpos::setRefSpeed(const double& value) {
    refSpeed = value;
}

// -----------------------------------------------------------------------------

void MotorIpos::setTr(const double& value) {
    tr = value;
}

// -----------------------------------------------------------------------------

bool MotorIpos::send(uint32_t cob, uint16_t len, uint8_t * msgData) {

    if ( (lastUsage - yarp::os::Time::now()) < DELAY )
        yarp::os::Time::delay( lastUsage + DELAY - yarp::os::Time::now() );

    if( ! canDevicePtr->sendRaw(cob + this->canId, len, msgData) )
        return false;

    lastUsage = yarp::os::Time::now();
    return true;
}

// -----------------------------------------------------------------------------

bool MotorIpos::start() {

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

bool MotorIpos::readyToSwitchOn() {

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

bool MotorIpos::switchOn() {

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

bool MotorIpos::enable() {

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

bool MotorIpos::interpretMessage( can_msg * message) {

    if( (message->data[1]==0x64) && (message->data[2]==0x60) )
    {
        //-- Commenting encoder value (response to petition) as way too verbose, happens all the time.
        CD_DEBUG("Got encoder value (response to petition). canId: %d (via %X).\n",canId,message->id-canId);
        int got;
        memcpy(&got, message->data+4,4);
        setEncoder( got / ( 11.11112 * getTr() ) );
        return true;
    }

    if( (message->id-canId) == 0x580 )  // SDO
    {
        if( (message->data[0]==0x60)&&(message->data[1]==0x7A) ) {
            CD_DEBUG("Got SDO ack \"position target\" from driver. canId: %d (via %X).\n",canId,message->id-canId);
        } else {
            CD_DEBUG("Got SDO ack from driver side: type not known: %X %X. canId: %d (via %X).\n",message->data[1],message->data[0],canId,message->id-canId);
        }
        return true;
    }

    if( (message->id-canId) == 0x180 )  // PDO1
    {
        if( (message->data[0]==0x37)&&(message->data[1]==0x92) ) {
            CD_DEBUG("Got PDO1 that it is observed as ack \"start position\" from driver. canId: %d (via %X).\n",canId,message->id-canId);
        } else if( (message->data[0]==0x37)&&(message->data[1]==0x86) ) {
            CD_DEBUG("Got PDO1 that it is observed when driver arrives to position target. canId: %d (via %X).\n",canId,message->id-canId);
        } else {
            CD_DEBUG("Got PDO1 from driver side: unknown, %X %X %X %X %X %X %X %X. canId: %d (via %X).\n",
                    message->data[0],message->data[1],
                    message->data[2],message->data[3],
                    message->data[4],message->data[5],
                    message->data[6],message->data[7],
                    canId,message->id-canId);
        }
        return true;
    }

    if( (message->id-canId) == 0x280 )  // PDO2
    {
        if( (message->data[0]==0x37)&&(message->data[1]==0x92) ) {
            CD_DEBUG("Got PDO2 that it is observed as ack \"start position\" from driver. canId: %d (via %X).\n",canId,message->id-canId);
        } else if( (message->data[0]==0x37)&&(message->data[1]==0x86) ) {
            CD_DEBUG("Got PDO2 that it is observed when driver arrives to position target. canId: %d (via %X).\n",canId,message->id-canId);
        } else {
            CD_DEBUG("Got PDO2 from driver side: unknown, %X %X %X %X %X %X %X %X. canId: %d (via %X).\n",
                    message->data[0],message->data[1],
                    message->data[2],message->data[3],
                    message->data[4],message->data[5],
                    message->data[6],message->data[7],
                    canId,message->id-canId);
        }
        return true;
    }

    if( (message->id-canId) == 0x80 )  // EMERGENCY (EMCY), Table 4.2 Emergency Error Codes (p57, 73/263)
    {
        CD_ERROR("Got EMERGENCY from iPOS: %X %X %X %X %X %X %X %X. canId: %d (via %X): ",
            message->data[0],message->data[1],
            message->data[2],message->data[3],
            message->data[4],message->data[5],
            message->data[6],message->data[7],
            canId,message->id-canId);
        if( (message->data[1]==0x00)&&(message->data[0]==0x00) ) {
            CD_ERROR_NO_HEADER("Error Reset or No Error.\n");
        } else if ( (message->data[1]==0x10)&&(message->data[0]==0x00) ) {
            CD_ERROR_NO_HEADER("Generic error.\n");
        } else if ( (message->data[1]==0x23)&&(message->data[0]==0x10) ) {
            CD_ERROR_NO_HEADER("Continuous over-current.\n");
        } else if ( (message->data[1]==0x23)&&(message->data[0]==0x40) ) {
            CD_ERROR_NO_HEADER("Short-circuit.\n");
        } else if ( (message->data[1]==0x32)&&(message->data[0]==0x10) ) {
            CD_ERROR_NO_HEADER("DC-link over-voltage.\n");
        } else if ( (message->data[1]==0x32)&&(message->data[0]==0x20) ) {
            CD_ERROR_NO_HEADER("DC-link under-voltage.\n");
        } else if ( (message->data[1]==0x42)&&(message->data[0]==0x80) ) {
            CD_ERROR_NO_HEADER("Over temperature motor.\n");
        } else if ( (message->data[1]==0x43)&&(message->data[0]==0x10) ) {
            CD_ERROR_NO_HEADER("Over temperature drive.\n");
        } else if ( (message->data[1]==0x54)&&(message->data[0]==0x41) ) {
            CD_ERROR_NO_HEADER("Driver disabled due to enable input.\n");
        } else if ( (message->data[1]==0x54)&&(message->data[0]==0x42) ) {
            CD_ERROR_NO_HEADER("Negative limit switch active.\n");
        } else if ( (message->data[1]==0x54)&&(message->data[0]==0x43) ) {
            CD_ERROR_NO_HEADER("Positive limit switch active.\n");
        } else if ( (message->data[1]==0x61)&&(message->data[0]==0x00) ) {
            CD_ERROR_NO_HEADER("Invalid setup data.\n");
        } else if ( (message->data[1]==0x75)&&(message->data[0]==0x00) ) {
            CD_ERROR_NO_HEADER("Communication error.\n");
        } else if ( (message->data[1]==0x81)&&(message->data[0]==0x10) ) {
            CD_ERROR_NO_HEADER("CAN overrun (message lost).\n");
        } else if ( (message->data[1]==0x81)&&(message->data[0]==0x30) ) {
            CD_ERROR_NO_HEADER("Life guard error or heartbeat error.\n");
        } else if ( (message->data[1]==0x83)&&(message->data[0]==0x31) ) {
            CD_ERROR_NO_HEADER("I2t protection triggered.\n");
        } else if ( (message->data[1]==0x85)&&(message->data[0]==0x80) ) {
            CD_ERROR_NO_HEADER("Position wraparound / Hal sensor missing.\n");
        } else if ( (message->data[1]==0x86)&&(message->data[0]==0x11) ) {
            CD_ERROR_NO_HEADER("Control error / Following error.\n");
        } else if ( (message->data[1]==0x90)&&(message->data[0]==0x00) ) {
            CD_ERROR_NO_HEADER("Command error\n");
        } else if ( (message->data[1]==0xFF)&&(message->data[0]==0x01) ) {
            CD_ERROR_NO_HEADER("Generic interpolated position mode error ( PVT / PT error.\n");
        } else if ( (message->data[1]==0xFF)&&(message->data[0]==0x02) ) {
            CD_ERROR_NO_HEADER("Change set acknowledge bit wrong value.\n");
        } else if ( (message->data[1]==0xFF)&&(message->data[0]==0x03) ) {
            CD_ERROR_NO_HEADER("Specified homing method not available.\n");
        } else if ( (message->data[1]==0xFF)&&(message->data[0]==0x04) ) {
            CD_ERROR_NO_HEADER("A wrong mode is set in object 6060h, modes_of_operation.\n");
        } else if ( (message->data[1]==0xFF)&&(message->data[0]==0x05) ) {
            CD_ERROR_NO_HEADER("Specified digital I/O line not available.\n");
        } else {
            CD_ERROR_NO_HEADER("NOT SPECIFIED IN MANUAL.\n");
        }

        return true;
    }

    //--------------- Debugged up to here -------------------------

    if( (message->data[0]==0x01)&&(message->data[1]==0xFF)&&(message->data[2]==0x01)&&(message->data[4]==0x20) )
    {
        ptBuffer.wait();
        CD_WARNING("pt buffer full! canId: %d.\n",canId);
        return true;
    }

    if( (message->data[0]==0x01)&&(message->data[1]==0xFF)&&(message->data[2]==0x01)&&(message->data[4]==0x00) )
    {
        ptBuffer.post();
        CD_WARNING("pt buffer empty. canId: %d.\n",canId);
        return true;
    }

    if( (message->data[0]==0x01)&&(message->data[1]==0xFF)&&(message->data[2]==0x01)&&(message->data[3]==0x08) )
    {
        CD_WARNING("pt buffer message (don't know what it means). canId: %d.\n",canId);
        return true;
    }

    if( (message->data[0]==0x37)&&(message->data[1]==0x96) )
    {
        CD_WARNING("Ended movement. canId: %d (via %X).\n",canId,message->id-canId);
        ptMovementDone = true;
        return true;
    }

    if( (message->data[1]==0x41)&&(message->data[2]=0x60) )
    {
        if(message->data[5] & 4) {
            targetReached = true;
            CD_DEBUG("Target reached. canId: %d.\n",canId);
        }
        else
        {
            targetReached = false;
            CD_DEBUG("Target NOT reached. canId: %d.\n",canId);
        }
        return true;
    }

    //-- error stuff

    //-- no meaning in next line
    //if (message->id>=0x80 && message->id<0x100) return true;

    if (message->data[0] == 0x01 && message->data[1] == 0xFF)
    {
        CD_DEBUG("PVT control message. canId: %d.\n",canId);
        return true;
    }


    CD_WARNING("Unknown message: %X %X %X %X %X %X %X %X, canId: %d (via %X)\n",
            message->data[0],message->data[1],
            message->data[2],message->data[3],
            message->data[4],message->data[5],
            message->data[6],message->data[7],
            canId,message->id-canId);

    return true;

}  //-- ends interpretMessage

// -----------------------------------------------------------------------------
