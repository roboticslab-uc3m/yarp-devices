// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "MotorIpos.hpp"

// -----------------------------------------------------------------------------

bool teo::MotorIpos::start() {

    //*************************************************************
    uint8_t msg_start[] = {0x01,0x01};

    msg_start[1]=this->canId;
    if( ! canDevicePtr->sendRaw(0, 2, msg_start) )
    {
        CD_ERROR("Could not send \"start\". %s\n", msgToStr(0, 2, msg_start).c_str() );
        return false;
    }
    CD_SUCCESS("Sent \"start\". %s\n", msgToStr(0, 2, msg_start).c_str() );
    //*************************************************************

    return true;
}

// -----------------------------------------------------------------------------

bool teo::MotorIpos::readyToSwitchOn() {

    //*************************************************************
    uint8_t msg_readyToSwitchOn[] = {0x06,0x00}; //-- readyToSwitchOn, also acts as shutdown.

    if( ! this->send( 0x200, 2, msg_readyToSwitchOn) )
    {
        CD_ERROR("Could not send \"readyToSwitchOn/shutdown\". %s\n", msgToStr(0x200, 2, msg_readyToSwitchOn).c_str() );
        return false;
    }
    CD_SUCCESS("Sent \"readyToSwitchOn/shutdown\". %s\n", msgToStr(0x200, 2, msg_readyToSwitchOn).c_str() );
    //*************************************************************

    return true;
}

// -----------------------------------------------------------------------------

bool teo::MotorIpos::switchOn() {

    //*************************************************************
    uint8_t msg_switchOn[] = {0x07,0x00};  //-- switchOn, also acts as disableOperation
    if( ! this->send( 0x200, 2, msg_switchOn) )
    {
        CD_ERROR("Could not send \"switchOn/disableOperation\". %s\n", msgToStr(0x200, 2, msg_switchOn).c_str() );
        return false;
    }
    CD_SUCCESS("Sent \"switchOn/disableOperation\". %s\n", msgToStr(0x200, 2, msg_switchOn).c_str() );
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
        CD_ERROR("Could not send \"enable\". %s\n", msgToStr(0x200, 2, msg_enable).c_str() );
        return false;
    }
    CD_SUCCESS("Sent \"enable\". %s\n", msgToStr(0x200, 2, msg_enable).c_str() );
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

std::string teo::MotorIpos::msgToStr(uint32_t cob, uint16_t len, uint8_t * msgData) {
    std::stringstream tmp;
    for(int i=0; i < len-1; i++)
    {
        tmp << std::hex << static_cast<int>(*(msgData+i)) << " ";
    }
    tmp << std::hex << static_cast<int>(*(msgData+len));
    tmp << ". canId(";
    tmp << std::dec << canId;
    tmp << ") via(";
    tmp << std::hex << cob;
    tmp << ").";
    return tmp.str();
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

void teo::MotorIpos::run() {

    while ( ! this->isStopping() ) {

        struct can_msg buffer;

    }  //-- ends: while ( ! this->isStopping() ).

    CD_INFO("Last thread run.\n");

    return;
}

// -----------------------------------------------------------------------------
