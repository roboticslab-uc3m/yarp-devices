// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "MotorIpos.hpp"

// -----------------------------------------------------------------------------

bool teo::MotorIpos::setCanBusPtr(CanBusHico *canDevicePtr) {

    this->canDevicePtr = canDevicePtr;
    CD_SUCCESS("Ok pointer to CAN bus device %d.\n",canId);

}

// -----------------------------------------------------------------------------

bool teo::MotorIpos::start() {

    this->getStartReady.wait();
    this->getStart = false;
    this->getStartReady.post();

    //*************************************************************
    uint8_t msg_start[] = {0x01,0x00};  // NMT

    msg_start[1]=this->canId;
    if( ! canDevicePtr->sendRaw(0, 2, msg_start) )
    {
        CD_ERROR("Could not send \"start\". %s\n", msgToStr(0, 2, msg_start).c_str() );
        return false;
    }
    CD_SUCCESS("Sent \"start\". %s\n", msgToStr(0, 2, msg_start).c_str() );
    //*************************************************************

    while( (! this->getStart) ) {
        CD_INFO("Waiting for response to \"start\" on id %d...\n", this->canId);
        yarp::os::Time::delay(0.1);  //-- [s]
    }

    return true;
}

// -----------------------------------------------------------------------------

bool teo::MotorIpos::readyToSwitchOn() {

    this->getReadyToSwitchOnReady.wait();
    this->getReadyToSwitchOn = false;
    this->getReadyToSwitchOnReady.post();

    //*************************************************************
    uint8_t msg_readyToSwitchOn[] = {0x06,0x00}; //-- readyToSwitchOn, also acts as shutdown.

    if( ! this->send( 0x200, 2, msg_readyToSwitchOn) )
    {
        CD_ERROR("Could not send \"readyToSwitchOn/shutdown\". %s\n", msgToStr(0x200, 2, msg_readyToSwitchOn).c_str() );
        return false;
    }
    CD_SUCCESS("Sent \"readyToSwitchOn/shutdown\". %s\n", msgToStr(0x200, 2, msg_readyToSwitchOn).c_str() );
    //*************************************************************

    while( (! this->getReadyToSwitchOn) ) {
        CD_INFO("Waiting for response to \"readyToSwitchOn/shutdown\" on id %d...\n", this->canId);
        yarp::os::Time::delay(0.1);  //-- [s]
    }

    return true;
}

// -----------------------------------------------------------------------------

bool teo::MotorIpos::switchOn() {

    this->getSwitchOnReady.wait();
    this->getSwitchOn = false;
    this->getSwitchOnReady.post();

    //*************************************************************
    uint8_t msg_switchOn[] = {0x07,0x00};  //-- switchOn, also acts as disableOperation
    if( ! this->send( 0x200, 2, msg_switchOn) )
    {
        CD_ERROR("Could not send \"switchOn/disableOperation\". %s\n", msgToStr(0x200, 2, msg_switchOn).c_str() );
        return false;
    }
    CD_SUCCESS("Sent \"switchOn/disableOperation\". %s\n", msgToStr(0x200, 2, msg_switchOn).c_str() );

    while( (! this->getSwitchOn) ) {
        CD_INFO("Waiting for response to \"switchOn/disableOperation\" on id %d...\n", this->canId);
        yarp::os::Time::delay(0.1);  //-- [s]
    }

    return true;
}

// -----------------------------------------------------------------------------

bool teo::MotorIpos::enable() {

    this->getEnableReady.wait();
    this->getEnable = false;
    this->getEnableReady.post();

    //*************************************************************
    uint8_t msg_enable[] = {0x0F,0x00}; // enable

    if( ! this->send( 0x200, 2, msg_enable) )
    {
        CD_ERROR("Could not send \"enable\". %s\n", msgToStr(0x200, 2, msg_enable).c_str() );
        return false;
    }
    CD_SUCCESS("Sent \"enable\". %s\n", msgToStr(0x200, 2, msg_enable).c_str() );
    //*************************************************************

    while( (! this->getEnable) ) {
        CD_INFO("Waiting for response to \"enable\" on id %d...\n", this->canId);
        yarp::os::Time::delay(0.1);  //-- [s]
    }

    return true;
}

// -----------------------------------------------------------------------------

bool teo::MotorIpos::recoverFromError() {

    //*************************************************************
    //j//uint8_t msg_recover[]={0x23,0xFF}; // Control word 6040

    //j//if( ! send(0x200, 2, msg_recover)){
    //j//    CD_ERROR("Sent \"recover\". %s\n", msgToStr(0x200, 2, msg_recover).c_str() );
    //j//    return false;
    //j//}
    //j//CD_SUCCESS("Sent \"recover\". %s\n", msgToStr(0x200, 2, msg_recover).c_str() );
    //*************************************************************

    return true;
}

// -----------------------------------------------------------------------------

bool teo::MotorIpos::interpretMessage( can_msg * message) {

    //--------------- Give high priority to PT, override EMGY red -------------------------

    if( (message->data[1]==0xFF)&&(message->data[2]==0x01)&&(message->data[4]==0x20) )
    {
        ptBuffer.wait();
        CD_WARNING("pt buffer full! canId: %d.\n",canId);
        return true;
    }
    else if( (message->data[1]==0xFF)&&(message->data[2]==0x01)&&(message->data[4]==0x00) )
    {
        ptBuffer.post();
        CD_WARNING("pt buffer empty. canId: %d.\n",canId);
        return true;
    }
    else if( (message->data[1]==0xFF)&&(message->data[2]==0x01)&&(message->data[3]==0x08) )
    {
        CD_WARNING("pt buffer message (don't know what it means). canId: %d.\n",canId);
        return true;
    }
    else if( (message->data[0]==0x37)&&(message->data[1]==0x96) )
    {
        CD_WARNING("pt movement ended. canId: %d (via %X).\n",canId,message->id-canId);
        ptMovementDone = true;
        return true;
    }
    else if (message->data[0] == 0x01 && message->data[1] == 0xFF)
    {
        CD_DEBUG("PVT control message. canId: %d.\n",canId);
        return true;
    }
    else if( (message->id-canId) == 0x580 )  // -------------- SDO ----------------------
    {
        if( (message->data[1]==0x64) && (message->data[2]==0x60) )  // Manual 6064h
        {
            //-- Commenting encoder value (response to petition) as way too verbose, happens all the time.
            //CD_DEBUG("Got encoder value (response to petition). %s\n",msgToStr(message).c_str());
            int got;
            memcpy(&got, message->data+4,4);
            encoderReady.wait();
            encoder =  got / ( 11.11112 * this->tr );
            encoderTimestamp = message->ts;
            encoderReady.post();
            return true;
        } else if( (message->data[1]==0x7A)&&(message->data[2]==0x60) ) {  // Manual 607Ah
            CD_DEBUG("Got SDO ack \"position target\" from driver. %s\n",msgToStr(message).c_str());
            return true;
        } else if( (message->data[1]==0x60)&&(message->data[2]==0x60) ) {  // Manual 6060h should behave like 6061h, but ack always says mode 0.
            CD_DEBUG("Got SDO ack \"modes of operation\" from driver. %s\n",msgToStr(message).c_str());
            return true;
        } else if( (message->data[1]==0x61)&&(message->data[2]==0x60) ) {  // Manual 6060h/6061h
            CD_DEBUG("Got SDO \"modes of operation display\" from driver. %s\n",msgToStr(message).c_str());
            int got;
            memcpy(&got, message->data+4,4);
            if(-5==got) {
                CD_DEBUG("\t-iPOS specific: External Reference Torque Mode.\n");
                getModeReady.wait();
                    getMode = VOCAB_TORQUE_MODE;
                getModeReady.post();
            } else if(-4==got) {
                CD_DEBUG("\t-iPOS specific: External Reference Speed Mode.\n");
                getModeReady.wait();
                    getMode = 0;
                getModeReady.post();
            } else if(-3==got) {
                CD_DEBUG("\t-iPOS specific: External Reference Position Mode.\n");
                getModeReady.wait();
                    getMode = 0;
                getModeReady.post();
            } else if(-2==got) {
                CD_DEBUG("\t-iPOS specific: Electronic Camming Position Mode.\n");
                getModeReady.wait();
                    getMode = 0;
                getModeReady.post();
            } else if(-1==got) {
                CD_DEBUG("\t-iPOS specific: Electronic Gearing Position Mode.\n");
                getModeReady.wait();
                    getMode = 0;
                getModeReady.post();
            } else if(1==got) {
                CD_DEBUG("\t-Profile Position Mode.\n");
                getModeReady.wait();
                    getMode = VOCAB_POSITION_MODE;
                getModeReady.post();
            } else if(3==got) {
                CD_DEBUG("\t-Profile Velocity Mode.\n");
                getModeReady.wait();
                    getMode = VOCAB_VELOCITY_MODE;
                getModeReady.post();
            } else if(6==got) {
                CD_DEBUG("\t-Homing Mode.\n");
                getModeReady.wait();
                    getMode = 0;
                getModeReady.post();
            } else if(7==got) {
                CD_DEBUG("\t-Interpolated Position Mode.\n");
                getModeReady.wait();
                    getMode = 0;
                getModeReady.post();
            } else {
                CD_WARNING("\t-Mode \"%d\" not specified in manual, may be in Fault or not enabled yet.\n",got);
                getModeReady.wait();
                    getMode = 0;
                getModeReady.post();
                return true;
            }
            return true;
        } else if( (message->data[1]==0x41)&&(message->data[2]==0x60) ) {  // Manual 6041h: Status word; Table 5.4 Bit Assignment in Status Word (also see 5.5)
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
                targetReachedReady.wait();
                    targetReached = true;
                targetReachedReady.post();
                CD_DEBUG("\t-Target reached.\n");  // true
            } else {
                CD_DEBUG("\t-Target not reached.\n");  // false (improvised, not in manual, but reasonable).
                targetReachedReady.wait();
                    targetReached = false;
                targetReachedReady.post();
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
            return true;
        } else if( (message->data[1]==0x00)&&(message->data[2]==0x20) ) {  // Manual 2000h: Motion Error Register
            CD_DEBUG("Got SDO ack \"Motion Error Register\" from driver. %s\n",msgToStr(message).c_str());

            if(message->data[4] & 1){//0000 0001 (bit 0)
                CD_DEBUG("\t*CAN error. Set when CAN controller is in error mode.\n");
            }
            if(message->data[4] & 2){//0000 0010 (bit 1)
                CD_DEBUG("\t*Short-circuit. Set when protection is triggered.\n");
            }
            if(message->data[4] & 4){//0000 0100 (bit 2)
                CD_DEBUG("\t*Invalid setup data. Set when the EEPROM stored setup data is not valid or not present.\n");
            }
            if(message->data[4] & 8){//0000 1000 (bit 3)
                CD_DEBUG("\t*Control error (position/speed error too big). Set when protection is triggered.\n");
            }
            if(message->data[4] & 16){//0001 0000 (bit 4)
                CD_DEBUG("\t*Communication error. Set when protection is triggered.\n");//true
            }
            if(message->data[4] & 32){//0010 0000 (bit 5)
                CD_DEBUG("\t*Motor position wraps around. Set when protection is triggered.\n");
            }
            if(message->data[4] & 64){//0100 0000 (bit 6)
                CD_DEBUG("\t*Positive limit switch active. Set when LSP input is in active state.\n");
            }
            if(message->data[4] & 128){//1000 0000 (bit 7)
                CD_DEBUG("\t*Negative limit switch active. Set when LSN input is in active state.\n");
            }
            if(message->data[5] & 1){//(bit 8)
                CD_DEBUG("\t*Over current. Set when protection is triggered.\n");
            }
            if(message->data[5] & 2){//(bit 9)
                CD_DEBUG("\t*I2t protection. Set when protection is triggered.\n");
            }
            if(message->data[5] & 4){//(bit 10)
                CD_DEBUG("\t*Over temperature motor. Set when protection is triggered.\n");
            }
            if(message->data[5] & 8){//(bit 11)
                CD_DEBUG("\t*Over temperature drive. Set when protection is triggered.\n");
            }
            if(message->data[5] & 16){//(bit 12)
                CD_DEBUG("\t*Over-voltage. Set when protection is triggered.\n");
            }
            if(message->data[5] & 32){//(bit 13)
                CD_DEBUG("\t*Under-voltage. Set when protection is triggered.\n");
            }
            if(message->data[5] & 64){//(bit 14)
                CD_DEBUG("\t*Command error. This bit is set in several situations. They can be distinguished either by the associated emergency code, or in conjunction with other bits.\n");
            }
            if(message->data[5] & 128){//(bit 15)
                CD_DEBUG("\t*Drive disabled due to enable input. Set when enable input is on disable state.\n");
            }
            return true;
        } else if( (message->data[1]==0x02)&&(message->data[2]==0x10) ) {  // Manual 1002h contains "6041h Status word" plus Table 5.6
            CD_DEBUG("Got \"manufacturer status register\" from driver. %s\n",msgToStr(message).c_str());

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
                CD_DEBUG("\t-Target reached.\n");  // true
            } else {
                CD_DEBUG("\t-Target not reached.\n");  // false (improvised, not in manual, but reasonable).
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
            if(message->data[6] & 1){//(bit 16)
                CD_DEBUG("\t*Drive/motor initialization performed.\n");
            }
            if(message->data[6] & 2){//(bit 17)
                CD_DEBUG("\t*Position trigger 1 reached.\n");
            }
            if(message->data[6] & 4){//(bit 18)
                CD_DEBUG("\t*Position trigger 2 reached.\n");
            }
            if(message->data[6] & 8){//(bit 19)
                CD_DEBUG("\t*Position trigger 3 reached.\n");
            }
            if(message->data[6] & 16){//(bit 20)
                CD_DEBUG("\t*Position trigger 4 reached.\n");
            }
            if(message->data[6] & 32){//(bit 21)
                CD_DEBUG("\t*AUTORUN mode enabled.\n");
            }
            if(message->data[6] & 64){//(bit 22)
                CD_DEBUG("\t*Limit switch positive event / interrupt triggered.\n");
            }
            if(message->data[6] & 128){//(bit 23)
                CD_DEBUG("\t*Limit switch negative event / interrupt triggered.\n");
            }
            if(message->data[7] & 1){//(bit 24)
                CD_DEBUG("\t*Capture event/interrupt triggered.\n");
            }
            if(message->data[7] & 2){//(bit 25)
                CD_DEBUG("\t*Target command reached.\n");
            }
            if(message->data[7] & 4){//(bit 26)
                CD_DEBUG("\t*Motor I2t protection warning level reached.\n");
            }
            if(message->data[7] & 8){//(bit 27)
                CD_DEBUG("\t*Drive I2t protection warning level reached.\n");
            }
            if(message->data[7] & 16){//(bit 28)
                CD_DEBUG("\t*Gear ratio in electronic gearing mode reached.\n");
            }
            if(message->data[7] & 64){//(bit 30)
                CD_DEBUG("\t*Reference position in absolute electronic camming mode reached.\n");
            }
            if(message->data[7] & 128){//(bit 31)
                CD_DEBUG("\t*Drive/motor in fault status.\n");
            }
            return true;
        } else if( (message->data[1]==0x02)&&(message->data[2]==0x20) ) {  // 2002h: Detailed Error Register
            CD_DEBUG("Got SDO ack \"Detailed Error Register\" from driver. %s\n",msgToStr(message).c_str());

            if(message->data[4] & 1){//0000 0001 (bit 0)
                CD_DEBUG("\t**The number of nested function calls exceeded the length of TML stack. Last function call was ignored.\n");
            }
            if(message->data[4] & 2){//0000 0010 (bit 1)
                CD_DEBUG("\t**A RET/RETI instruction was executed while no function/ISR was active.\n");
            }
            if(message->data[4] & 4){//0000 0100 (bit 2)
                CD_DEBUG("\t**A call to an inexistent homing routine was received.\n");
            }
            if(message->data[4] & 8){//0000 1000 (bit 3)
                CD_DEBUG("\t**A call to an inexistent function was received.\n");
            }
            if(message->data[4] & 16){//0001 0000 (bit 4)
                CD_DEBUG("\t**UPD instruction received while AXISON was executed. The UPD instruction was ingnored and it must be sent again when AXISON is completed.\n");
            }
            if(message->data[4] & 32){//0010 0000 (bit 5)
                CD_DEBUG("\t**Cancelable call instruction received while another cancelable function was active.\n");
            }
            if(message->data[4] & 64){//0100 0000 (bit 6)
                CD_DEBUG("\t**Positive software limit switch is active.\n");
            }
            if(message->data[4] & 128){//1000 0000 (bit 7)
                CD_DEBUG("\t**Negative software limit switch is active.\n");
            }
            if(message->data[5] & 1){//(bit 8)
                CD_DEBUG("\t**S-curve parameters caused and invalid profile. UPD instruction was ignored.\n");
            }
            if(message->data[5] & 2){//(bit 9)
                CD_DEBUG("\t**Update ignored for S-curve.\n");
            }
            if(message->data[5] & 4){//(bit 10)
                CD_DEBUG("\t**Encoder broken wire.\n");
            }
            if(message->data[5] & 8){//(bit 11)
                CD_DEBUG("\t**Motionless start failed.\n");
            }
            if(message->data[5] & 32){//(bit 13)
                CD_DEBUG("\t**Self check error.\n");
            }
            return true;
        } else if( (message->data[1]==0x83)&&(message->data[2]==0x60) ) {  // Manual 6083h: Profile acceleration
            CD_DEBUG("Got SDO ack \"posmode_acc\" from driver. %s\n",msgToStr(message).c_str());
            return true;
        } else if( (message->data[1]==0x81)&&(message->data[2]==0x60) ) {  // Manual 6081h: Profile velocity
            CD_DEBUG("Got SDO ack \"posmode_speed\" from driver. %s\n",msgToStr(message).c_str());
            return true;
        } else if( (message->data[1]==0x7D)&&(message->data[2]==0x60) ) {  // Manual 607Dh: Software position limit
            if (message->data[3]==0x01) {
                CD_DEBUG("Got SDO ack \"msg_position_min\" from driver. %s\n",msgToStr(message).c_str());
            } else if (message->data[3]==0x02) {
                CD_DEBUG("Got SDO ack \"msg_position_max\" from driver. %s\n",msgToStr(message).c_str());
            } else {
                CD_WARNING("Got SDO ack \"msg_position_????\" from driver. %s\n",msgToStr(message).c_str());
                return false;
            }
            return true;
        } else if( (message->data[1]==0x81)&&(message->data[2]==0x20) ) {  // Manual 2081h: Set/Change the actual motor position
            CD_DEBUG("Got SDO ack \"set encoder\" from driver. %s\n",msgToStr(message).c_str());
            return true;
        } else if( (message->data[1]==0x02)&&(message->data[2]==0x16) ) {  // Manual 1602h: Receive PDO3 Mapping Parameters
            CD_DEBUG("Got SDO ack \"RPDO3 changes\" from driver. %s\n",msgToStr(message).c_str());
            return true;
        } else if( (message->data[1]==0xC0)&&(message->data[2]==0x60) ) {  // Manual 60C0h: Interpolation sub mode select
            CD_DEBUG("Got SDO ack \"Interpolation sub mode select.\" from driver. %s\n",msgToStr(message).c_str());
            return true;
        } else if( (message->data[1]==0x74)&&(message->data[2]==0x20) ) {  // Manual 2074h: Interpolated position buffer configuration
            CD_DEBUG("Got SDO ack \"Interpolated position buffer configuration.\" from driver. %s\n",msgToStr(message).c_str());
            return true;
        } else if( (message->data[1]==0x79)&&(message->data[2]==0x20) ) {  // Manual 2079h: Interpolated position initial position
            CD_DEBUG("Got SDO ack \"Interpolated position initial position.\" from driver. %s\n",msgToStr(message).c_str());
            return true;
        }
        CD_DEBUG("Got SDO ack from driver side: type not known. %s\n",msgToStr(message).c_str());
        return false;
    }
    else if( (message->id-canId) == 0x180 )  // ---------------------- PDO1 ----------------------
    {
        if( (message->data[0]==0x37)&&(message->data[1]==0x92) ) {
            CD_DEBUG("Got PDO1 that it is observed as ack \"start position\" from driver. %s\n",msgToStr(message).c_str());
            return true;
        } else if( (message->data[0]==0x37)&&(message->data[1]==0x86) ) {
            CD_DEBUG("Got PDO1 that it is observed when driver arrives to position target. %s\n",msgToStr(message).c_str());
            return true;
        } else if( (message->data[0]==0x40)&&(message->data[1]==0x02) ) {
            CD_DEBUG("Got PDO1 that it is observed as TRANSITION performed upon \"start\". %s\n",msgToStr(message).c_str());
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
        } else if( (message->data[0]==0x40)&&(message->data[1]==0x02) ) {
            CD_DEBUG("Got PDO2 that it is observed as TRANSITION performed upon \"start\". %s\n",msgToStr(message).c_str());
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
                CD_ERROR_NO_HEADER("Generic interpolated position mode error (PVT / PT error).\n");
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

    CD_ERROR("Unknown message: %s\n", msgToStr(message).c_str());

    return false;

}  //-- ends interpretMessage

// -----------------------------------------------------------------------------
