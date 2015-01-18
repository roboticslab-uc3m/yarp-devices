// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "MotorIpos.hpp"

// -----------------------------------------------------------------------------

teo::MotorIpos::MotorIpos(CanBusHico *canDevicePtr, const int& canId, const double &tr) : max(0), min(0), refAcceleration(0), refSpeed(0), encoder(0) {
    this->canDevicePtr = canDevicePtr;
    this->canId = canId;
    this->tr = tr;
    this->ptPointCounter = 0;
    this->ptMovementDone = false;
    this->targetReached = false;
    CD_SUCCESS("Created driver with canId %d and tr %f, pointer to CAN bus device, and all local parameters set to 0.\n",canId,tr);
}

// -----------------------------------------------------------------------------

int teo::MotorIpos::getCanId() {
    return canId;
}

// -----------------------------------------------------------------------------

double teo::MotorIpos::getEncoder() {
    double value;
    encoderReady.wait();
    value = encoder;
    encoderReady.post();
    return value;
}

// -----------------------------------------------------------------------------

double teo::MotorIpos::getMax() {
    return max;
}

// -----------------------------------------------------------------------------

double teo::MotorIpos::getMin() {
    return min;
}

// -----------------------------------------------------------------------------

double teo::MotorIpos::getRefAcceleration() {
    return refAcceleration;
}

// -----------------------------------------------------------------------------

double teo::MotorIpos::getRefSpeed() {
    return refSpeed;
}

// -----------------------------------------------------------------------------

double teo::MotorIpos::getTr() {
    return tr;
}

// -----------------------------------------------------------------------------

void teo::MotorIpos::setEncoder(const double& value) {
    encoderReady.wait();
    encoder = value;
    encoderReady.post();
}

// -----------------------------------------------------------------------------

void teo::MotorIpos::setMax(const double& value) {
    max = value;
}

// ----------------------------.-------------------------------------------------

void teo::MotorIpos::setMin(const double& value) {
    min = value;
}

// -----------------------------------------------------------------------------

void teo::MotorIpos::setRefAcceleration(const double& value) {
    refAcceleration = value;
}

// -----------------------------------------------------------------------------

void teo::MotorIpos::setRefSpeed(const double& value) {
    refSpeed = value;
}

// -----------------------------------------------------------------------------

void teo::MotorIpos::setTr(const double& value) {
    tr = value;
}

// -----------------------------------------------------------------------------

bool teo::MotorIpos::send(uint32_t cob, uint16_t len, uint8_t * msgData) {

    if ( (lastUsage - yarp::os::Time::now()) < DELAY )
        yarp::os::Time::delay( lastUsage + DELAY - yarp::os::Time::now() );

    if( ! canDevicePtr->sendRaw(cob + this->canId, len, msgData) )
        return false;

    lastUsage = yarp::os::Time::now();
    return true;
}

// -----------------------------------------------------------------------------

bool teo::MotorIpos::start() {

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

bool teo::MotorIpos::readyToSwitchOn() {

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

bool teo::MotorIpos::switchOn() {

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

bool teo::MotorIpos::enable() {

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

bool teo::MotorIpos::interpretMessage( can_msg * message) {

    if( (message->id-canId) == 0x580 )  // SDO
    {
        if( (message->data[1]==0x64) && (message->data[2]==0x60) )  // Manual 6064h
        {
            //-- Commenting encoder value (response to petition) as way too verbose, happens all the time.
            //CD_DEBUG("Got encoder value (response to petition). %s\n",msgToStr(message).c_str());
            int got;
            memcpy(&got, message->data+4,4);
            setEncoder( got / ( 11.11112 * getTr() ) );
            return true;
        } else if( (message->data[1]==0x7A)&&(message->data[2]==0x60) ) {  // Manual 607Ah
            CD_DEBUG("Got SDO ack \"position target\" from driver. %s\n",msgToStr(message).c_str());
            return true;
        }
        CD_DEBUG("Got SDO ack from driver side: type not known. %s\n",msgToStr(message).c_str());
        return false;
    }
    else if( (message->id-canId) == 0x180 )  // PDO1
    {
        if( (message->data[0]==0x37)&&(message->data[1]==0x92) ) {
            CD_DEBUG("Got PDO1 that it is observed as ack \"start position\" from driver. %s\n",msgToStr(message).c_str());
            return true;
        } else if( (message->data[0]==0x37)&&(message->data[1]==0x86) ) {
            CD_DEBUG("Got PDO1 that it is observed when driver arrives to position target. %s\n",msgToStr(message).c_str());
            return true;
        }
        CD_DEBUG("Got PDO1 from driver side: unknown. %s\n",msgToStr(message).c_str());
        return false;
    }
    else if( (message->id-canId) == 0x280 )  // PDO2
    {
        if( (message->data[0]==0x37)&&(message->data[1]==0x92) ) {
            CD_DEBUG("Got PDO2 that it is observed as ack \"start position\" from driver. %s\n",msgToStr(message).c_str());
            return true;
        } else if( (message->data[0]==0x37)&&(message->data[1]==0x86) ) {
            CD_DEBUG("Got PDO2 that it is observed when driver arrives to position target. %s\n",msgToStr(message).c_str());
            return true;
        }
        CD_DEBUG("Got PDO2 from driver side: unknown. %s\n",msgToStr(message).c_str());
        return false;
    }
    else if( (message->id-canId) == 0x80 )  // EMERGENCY (EMCY), Table 4.2 Emergency Error Codes (p57, 73/263)
    {
        CD_ERROR("Got EMERGENCY from iPOS. %s ",msgToStr(message).c_str());
        if( (message->data[1]==0x00)&&(message->data[0]==0x00) ) {
            CD_ERROR_NO_HEADER("Error Reset or No Error.\n");
            return true;
        } else if ( (message->data[1]==0x10)&&(message->data[0]==0x00) ) {
            CD_ERROR_NO_HEADER("Generic error.\n");
            return true;
        } else if (message->data[1]==0x23) {
            if (message->data[0]==0x10) {
                CD_ERROR_NO_HEADER("Continuous over-current.\n");
                return true;
            } else if (message->data[0]==0x40) {
                CD_ERROR_NO_HEADER("Short-circuit.\n");
                return true;
            }
            CD_ERROR_NO_HEADER("NOT SPECIFIED IN MANUAL.\n");
            return false;
        } else if (message->data[1]==0x32) {
            if (message->data[0]==0x10) {
                CD_ERROR_NO_HEADER("DC-link over-voltage.\n");
                return true;
            } else if (message->data[0]==0x20) {
                CD_ERROR_NO_HEADER("DC-link under-voltage.\n");
                return true;
            }
            CD_ERROR_NO_HEADER("NOT SPECIFIED IN MANUAL.\n");
            return false;
        } else if ( (message->data[1]==0x42)&&(message->data[0]==0x80) ) {
            CD_ERROR_NO_HEADER("Over temperature motor.\n");
            return true;
        } else if ( (message->data[1]==0x43)&&(message->data[0]==0x10) ) {
            CD_ERROR_NO_HEADER("Over temperature drive.\n");
            return true;
        } else if (message->data[1]==0x54) {
            if (message->data[0]==0x41) {
                CD_ERROR_NO_HEADER("Driver disabled due to enable input.\n");
                return true;
            } else if (message->data[0]==0x42) {
                CD_ERROR_NO_HEADER("Negative limit switch active.\n");
                return true;
            } else if (message->data[0]==0x43) {
                CD_ERROR_NO_HEADER("Positive limit switch active.\n");
                return true;
            }
            CD_ERROR_NO_HEADER("NOT SPECIFIED IN MANUAL.\n");
            return false;
        } else if ( (message->data[1]==0x61)&&(message->data[0]==0x00) ) {
            CD_ERROR_NO_HEADER("Invalid setup data.\n");
            return true;
        } else if ( (message->data[1]==0x75)&&(message->data[0]==0x00) ) {
            CD_ERROR_NO_HEADER("Communication error.\n");
            return true;
        } else if (message->data[1]==0x81) {
            if (message->data[0]==0x10) {
                CD_ERROR_NO_HEADER("CAN overrun (message lost).\n");
                return true;
            } else if (message->data[0]==0x30) {
                CD_ERROR_NO_HEADER("Life guard error or heartbeat error.\n");
                return true;
            }
            CD_ERROR_NO_HEADER("NOT SPECIFIED IN MANUAL.\n");
            return false;
        } else if ( (message->data[1]==0x83)&&(message->data[0]==0x31) ) {
            CD_ERROR_NO_HEADER("I2t protection triggered.\n");
            return true;
        } else if ( (message->data[1]==0x85)&&(message->data[0]==0x80) ) {
            CD_ERROR_NO_HEADER("Position wraparound / Hal sensor missing.\n");
            return true;
        } else if ( (message->data[1]==0x86)&&(message->data[0]==0x11) ) {
            CD_ERROR_NO_HEADER("Control error / Following error.\n");
            return true;
        } else if ( (message->data[1]==0x90)&&(message->data[0]==0x00) ) {
            CD_ERROR_NO_HEADER("Command error\n");
            return true;
        } else if (message->data[1]==0xFF) {
            if (message->data[0]==0x01) {
                CD_ERROR_NO_HEADER("Generic interpolated position mode error ( PVT / PT error.\n");
                return true;
            } else if (message->data[0]==0x02) {
                CD_ERROR_NO_HEADER("Change set acknowledge bit wrong value.\n");
                return true;
            } else if (message->data[0]==0x03) {
                CD_ERROR_NO_HEADER("Specified homing method not available.\n");
                return true;
            } else if (message->data[0]==0x04) {
                CD_ERROR_NO_HEADER("A wrong mode is set in object 6060h, modes_of_operation.\n");
                return true;
            } else if (message->data[0]==0x05) {
                CD_ERROR_NO_HEADER("Specified digital I/O line not available.\n");
                return true;
            }
            CD_ERROR_NO_HEADER("NOT SPECIFIED IN MANUAL.\n");
            return false;
        }
        CD_ERROR_NO_HEADER("NOT SPECIFIED IN MANUAL.\n");
        return false;
    }

    //--------------- Debugged up to here -------------------------

    if ((message->data[1]== 0x41) && (message->data[2]=0x60)) {  // Manual 6041h: Status word; Table 5.4 Bit Assignment in Status Word (also see 5.5)
        CD_DEBUG("Got \"status word\" from driver. %s\n",msgToStr(message).c_str());

        if(message->data[4] & 1){//0000 0001 (bit 0)
            CD_DEBUG("\t-Ready to switch on.\n");
        }
        if(message->data[4] & 2){//0000 0010 (bit 1)
            CD_DEBUG("\t-Switched on.\n");
        }
        if(message->data[4] & 4){//0000 0100 (bit 2)
            CD_DEBUG("\t-Operation Enabled.\n");
        }
        if(message->data[4] & 8){//0000 1000 (bit 3)
            CD_DEBUG("\t-Fault. If set, a fault condition is or was present in the drive.\n");
        }
        if(message->data[4] & 16){//0001 0000 (bit 4)
            CD_DEBUG("\t-Motor supply voltage is present.\n");//true
        } else {
            CD_DEBUG("\t-Motor supply voltage is absent.\n");//false
        }
        if(!(message->data[4] & 32)){//0010 0000 (bit 5), negated.
            CD_DEBUG("\t-Performing a quick stop.\n");
        }
        if(message->data[4] & 64){//0100 0000 (bit 6)
            CD_DEBUG("\t-Switch on disabled.\n");
        }
        if(message->data[4] & 128){//1000 0000 (bit 7)
            CD_DEBUG("\t-Warning. A TML function / homing was called, while another TML function / homing is still in execution. The last call is ignored.\n");
        }
        if(message->data[5] & 1){//(bit 8)
            CD_DEBUG("\t-A TML function or homing is executed. Until the function or homing execution ends or is aborted, no other TML function / homing may be called.\n");
        }
        if(message->data[5] & 2){//(bit 9)
            CD_DEBUG("\t-Remote: drive parameters may be modified via CAN and the drive will execute the command message.\n"); // true
        } else {
            CD_DEBUG("\t-Remote: drive is in local mode and will not execute the command message (only TML internal)."); // false
        }
        if(message->data[5] & 4){//(bit 10)
            CD_DEBUG("\t-Target reached.\n");
        }
        if(message->data[5] & 8){//(bit 11)
            CD_DEBUG("\t-Internal Limit Active.\n");
        }
        if(message->data[5] & 64){//(bit 14)
            CD_DEBUG("\t-Last event set has ocurred.\n"); // true
        } else {
            CD_DEBUG("\t-No event set or the programmed event has not occurred yet.\n"); // false
        }
        if(message->data[5] & 128){//(bit 15)
            CD_DEBUG("\t-Axis on. Power stage is enabled. Motor control is performed.\n"); // true
        } else {
            CD_DEBUG("\t-Axis off. Power stage is disabled. Motor control is not performed.\n"); // false
        }

        ////Much much more in Table 5.6

    }

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
        CD_WARNING("pt movement ended. canId: %d (via %X).\n",canId,message->id-canId);
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

    if (message->data[0] == 0x01 && message->data[1] == 0xFF)
    {
        CD_DEBUG("PVT control message. canId: %d.\n",canId);
        return true;
    }

    //-- no meaning in next line
    //if (message->id>=0x80 && message->id<0x100) return true;

    CD_WARNING("Unknown message: %s\n", msgToStr(message).c_str());

    return false;

}  //-- ends interpretMessage

// -----------------------------------------------------------------------------

std::string teo::MotorIpos::msgToStr(can_msg* message) {
    std::stringstream tmp;
    for(int i=0; i < message->dlc-1; i++)
    {
        tmp << std::hex << static_cast<int>(message->data[i]) << " ";
    }
    tmp << std::hex << static_cast<int>(message->data[message->dlc-1]);
    tmp << ". canId(";
    tmp << std::dec << (message->id & 0x7F);
    tmp << ") via(";
    tmp << std::hex << (message->id & 0xFF80);
    tmp << ").";
    return tmp.str();
}

// -----------------------------------------------------------------------------
