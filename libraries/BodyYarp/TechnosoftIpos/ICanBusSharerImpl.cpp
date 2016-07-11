// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TechnosoftIpos.hpp"

// -----------------------------------------------------------------------------

bool teo::TechnosoftIpos::setCanBusPtr(CanBusHico *canDevicePtr)
{

    this->canDevicePtr = canDevicePtr;
    CD_SUCCESS("Ok pointer to CAN bus device %d.\n",canId);

    return true;
}

// -----------------------------------------------------------------------------

bool teo::TechnosoftIpos::setIEncodersTimedRawExternal(IEncodersTimedRaw * iEncodersTimedRaw)
{

    this->iEncodersTimedRawExternal = iEncodersTimedRaw;

    CD_SUCCESS("Ok pointer to external encoder interface %p (%d). Updating with latest external...\n",iEncodersTimedRaw,canId);

    double v = sqrt(-1);  //-- v is a NaN
    CD_INFO("canId(%d) wait to get external encoder value...\n",this->canId);
    while( v != v )  //-- loop while v is still a NaN
    {
        iEncodersTimedRawExternal->getEncoderRaw(0,&v);
        //CD_INFO("Wait to get external encoder value...\n"); //\todo{activate these lines if blocking is too much}
        //Time::delay(0.2);
    }
    this->setEncoderRaw(0,v);  //-- Forces the relative encoder to this value.

    return true;
}

// -----------------------------------------------------------------------------
/** -- Start Remote Node: Used to change NMT state of one or all NMT slaves to Operational.
 PDO communication will beallowed. */

bool teo::TechnosoftIpos::start()
{

    uint8_t msg_start[] = {0x01,0x00};  // NMT Start Remote Node (to operational, Fig 4.1)

    msg_start[1]=this->canId; // -- in byte 1 it writes canId
    if( ! canDevicePtr->sendRaw(0, 2, msg_start) ) // -- manda el mensaje
    {
        CD_ERROR("Could not send \"start\". %s\n", msgToStr(0, 2, msg_start).c_str() );
        return false;
    }
    CD_SUCCESS("Sent \"start\". %s\n", msgToStr(0, 2, msg_start).c_str() );

    //-- Do not force expect response as only happens upon transition.
    //-- For example, if already started, function would get stuck.

    return true;
}

// -----------------------------------------------------------------------------

bool teo::TechnosoftIpos::readyToSwitchOn()
{

    uint8_t msg_readyToSwitchOn[] = {0x06,0x00}; //-- readyToSwitchOn, also acts as shutdown.
    // -- send se diferencia de senRaw en que tiene un delay y adems incluye el ID (mirar funcin)
    if( ! this->send( 0x200, 2, msg_readyToSwitchOn) ) // -- 0x200 (valor crtico que se pone sin saber qu significa) 2 (tamao del mensaje)
    {
        CD_ERROR("Could not send \"readyToSwitchOn/shutdown\". %s\n", msgToStr(0x200, 2, msg_readyToSwitchOn).c_str() );
        return false;
    }
    CD_SUCCESS("Sent \"readyToSwitchOn/shutdown\". %s\n", msgToStr(0x200, 2, msg_readyToSwitchOn).c_str() );

    //-- Do not force expect response as only happens upon transition.
    //-- For example, if already on readyToSwitchOn, function would get stuck.

    return true;
}

// -----------------------------------------------------------------------------

bool teo::TechnosoftIpos::switchOn()
{

    this->getSwitchOnReady.wait();
    this->getSwitchOn = false;
    this->getSwitchOnReady.post();

    uint8_t msg_switchOn[] = {0x07,0x00};  //-- switchOn, also acts as disableOperation
    if( ! this->send( 0x200, 2, msg_switchOn) )
    {
        CD_ERROR("Could not send \"switchOn/disableOperation\". %s\n", msgToStr(0x200, 2, msg_switchOn).c_str() );
        return false;
    }
    CD_SUCCESS("Sent \"switchOn/disableOperation\". %s\n", msgToStr(0x200, 2, msg_switchOn).c_str() );

    //while( (! this->getSwitchOn) ) {
    //    CD_INFO("Waiting for response to \"switchOn/disableOperation\" on id %d...\n", this->canId);
    //    yarp::os::Time::delay(0.1);  //-- [s]
    //}

    return true;
}

// -----------------------------------------------------------------------------

bool teo::TechnosoftIpos::enable()
{

    this->getEnableReady.wait();
    this->getEnable = false;
    this->getEnableReady.post();

    uint8_t msg_enable[] = {0x0F,0x00}; // enable

    if( ! this->send( 0x200, 2, msg_enable) )
    {
        CD_ERROR("Could not send \"enable\". %s\n", msgToStr(0x200, 2, msg_enable).c_str() );
        return false;
    }
    CD_SUCCESS("Sent \"enable\". %s\n", msgToStr(0x200, 2, msg_enable).c_str() );
    //*************************************************************

    //while( (! this->getEnable) ) {
    //    CD_INFO("Waiting for response to \"enable\" on id %d...\n", this->canId);
    //    yarp::os::Time::delay(0.1);  //-- [s]
    //}

    return true;
}


// -----------------------------------------------------------------------------

bool teo::TechnosoftIpos::recoverFromError()
{

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

/** Manual: 4.1.2. Device control
    Reset Node: The NMT master sets the state of the selected NMT slave to the reset application sub-state.
    In this state the drives perform a software reset and enter the pre-operational state.
 **/

bool teo::TechnosoftIpos::resetNodes()
{

    // NMT Reset Node (Manual 4.1.2.3)
    uint8_t msg_resetNodes[] = {0x81,0x00};  // reset all nodes ([0x00] = broadcast)

    //msg_resetNode[1]=this->canId; // -- It writes canId in byte 1
    if( ! canDevicePtr->sendRaw(0, 2, msg_resetNodes) ) // -- 0 (hace referencia al ID. Si est en 0 es como un broadcast) 2 (tamao del mensaje)
    {
        CD_ERROR("Could not send \"reset node\". %s\n", msgToStr(0, 2, msg_resetNodes).c_str() );
        return false;
    }
    CD_SUCCESS("Sent \"reset nodes\". %s\n", msgToStr(0, 2, msg_resetNodes).c_str() );

    //-- Do not force expect response as only happens upon transition.
    //-- For example, if already started, function would get stuck.

    return true;
}

/** Manual: 4.1.2. Device control
 * The NMT master sets the state of the selected NMT slave to the “reset communication” sub-state.
 * In this state the drives resets their communication and enter the pre-operational state.
 */

bool teo::TechnosoftIpos::resetCommunication()
{

    uint8_t msg_resetCommunication[] = {0x82,0x00};  // NMT Reset Communications (Manual 4.1.2.2)

    //msg_resetNode[1]=this->canId; // -- It writes canId in byte 1
    if( ! this->send(0x200, 2, msg_resetCommunication) ) // -- 0 (hace referencia al ID. Si est en 0 es como un broadcast) 2 (tamao del mensaje)
    {
        CD_ERROR("Could not send \"reset communication\". %s\n", msgToStr(0, 2, msg_resetCommunication).c_str() );
        return false;
    }
    CD_SUCCESS("Sent \"reset communication\". %s\n", msgToStr(0, 2, msg_resetCommunication).c_str() );

    //-- Do not force expect response as only happens upon transition.
    //-- For example, if already started, function would get stuck.

    return true;
}


/** Manual: 4.1.2. Device control
    Reset Node: The NMT master sets the state of the selected NMT slave to the reset application sub-state.
    In this state the drives perform a software reset and enter the pre-operational state.
 **/

bool teo::TechnosoftIpos::resetNode(int id)
{

    uint8_t msg_resetNode[] = {0x81,0x00};  // NMT Reset Node (Manual 4.1.2.3)

    msg_resetNode[1] = id; // -- It writes canId in byte 1
    if( ! canDevicePtr->sendRaw(0, 2, msg_resetNode) ) // -- 0 (hace referencia al ID. Si est en 0 es como un broadcast) 2 (tamao del mensaje)
    {
        CD_ERROR("Could not send \"reset node\". %s\n", msgToStr(0, 2, msg_resetNode).c_str() );
        return false;
    }
    CD_SUCCESS("Sent \"reset node\". %s\n", msgToStr(0, 2, msg_resetNode).c_str() );

    //-- Do not force expect response as only happens upon transition.
    //-- For example, if already started, function would get stuck.

    return true;
}



// -----------------------------------------------------------------------------

bool teo::TechnosoftIpos::interpretMessage( can_msg * message)
{

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
        CD_INFO("PVT control message. canId: %d.\n",canId);
        return true;
    }
    else if( (message->id-canId) == 0x580 )  // -------------- SDO ----------------------
    {
        if( (message->data[1]==0x64) && (message->data[2]==0x60) )  // Manual 6064h
        {
            //-- Commenting encoder value (response to petition) as way too verbose, happens all the time.
            //CD_INFO("Got encoder value (response to petition). %s\n",msgToStr(message).c_str());
            int got;
            memcpy(&got, message->data+4,4);
            encoderReady.wait();
            encoder =  got / ( (4096/360) * this->tr );
            encoderTimestamp = message->ts;
            encoderReady.post();
            return true;
        }
        else if( (message->data[1]==0x7E) && (message->data[2]==0x20) )     // Manual 207Eh
        {
            //-- Commenting torque value (response to petition) as way too verbose, happens all the time.
            //CD_INFO("Got torque value (response to petition). %s\n",msgToStr(message).c_str());
            int16_t got;
            memcpy(&got, message->data+4,2);
            getTorqueReady.wait();
            getTorque = got * (2.0 * 10.0) / 65520.0;
            getTorqueReady.post();
            return true;
        }
        else if( (message->data[1]==0x7A)&&(message->data[2]==0x60) )      // Manual 607Ah
        {
            CD_INFO("Got SDO ack \"position target\" from driver. %s\n",msgToStr(message).c_str());
            return true;
        }
        else if( (message->data[1]==0x60)&&(message->data[2]==0x60) )      // Manual 6060h should behave like 6061h, but ack always says mode 0.
        {
            CD_INFO("Got SDO ack \"modes of operation\" from driver. %s\n",msgToStr(message).c_str());
            return true;
        }
        else if( (message->data[1]==0x61)&&(message->data[2]==0x60) )      // Manual 6060h/6061h
        {
            CD_INFO("Got SDO \"modes of operation display\" from driver. %s\n",msgToStr(message).c_str());
            int got;
            memcpy(&got, message->data+4,4);
            if(-5==got)
            {
                CD_INFO("\t-iPOS specific: External Reference Torque Mode. canId: %d.\n",canId);
                getModeReady.wait();
                getMode = VOCAB_CM_TORQUE;
                getModeReady.post();
            }
            else if(-4==got)
            {
                CD_INFO("\t-iPOS specific: External Reference Speed Mode. canId: %d.\n",canId);
                getModeReady.wait();
                getMode = 0;
                getModeReady.post();
            }
            else if(-3==got)
            {
                CD_INFO("\t-iPOS specific: External Reference Position Mode. canId: %d.\n",canId);
                getModeReady.wait();
                getMode = 0;
                getModeReady.post();
            }
            else if(-2==got)
            {
                CD_INFO("\t-iPOS specific: Electronic Camming Position Mode. canId: %d.\n",canId);
                getModeReady.wait();
                getMode = 0;
                getModeReady.post();
            }
            else if(-1==got)
            {
                CD_INFO("\t-iPOS specific: Electronic Gearing Position Mode. canId: %d.\n",canId);
                getModeReady.wait();
                getMode = 0;
                getModeReady.post();
            }
            else if(1==got)
            {
                CD_INFO("\t-Profile Position Mode. canId: %d.\n",canId);
                getModeReady.wait();
                getMode = VOCAB_CM_POSITION;
                getModeReady.post();
            }
            else if(3==got)
            {
                CD_INFO("\t-Profile Velocity Mode. canId: %d.\n",canId);
                getModeReady.wait();
                getMode = VOCAB_CM_VELOCITY;
                getModeReady.post();
            }
            else if(6==got)
            {
                CD_INFO("\t-Homing Mode. canId: %d.\n",canId);
                getModeReady.wait();
                getMode = 0;
                getModeReady.post();
            }
            else if(7==got)
            {
                CD_INFO("\t-Interpolated Position Mode. canId: %d.\n",canId);
                getModeReady.wait();
                getMode = VOCAB_CM_POSITION_DIRECT;
                getModeReady.post();
            }
            else
            {
                CD_WARNING("\t-Mode \"%d\" not specified in manual, may be in Fault or not enabled yet. canId(%d).\n",got,(message->id & 0x7F));
                getModeReady.wait();
                getMode = VOCAB_FAILED;
                getModeReady.post();
                return true;
            }
            return true;
        }
        else if( (message->data[1]==0x41)&&(message->data[2]==0x60) )      // Manual 6041h: Status word; Table 5.4 Bit Assignment in Status Word (also see 5.5)
        {
            CD_INFO("Got \"status word\" from driver. %s\n",msgToStr(message).c_str());

            if(message->data[4] & 1) //0000 0001 (bit 0)
            {
                CD_INFO("\t-Ready to switch on. canId: %d.\n",canId);
            }
            if(message->data[4] & 2) //0000 0010 (bit 1)
            {
                CD_INFO("\t-Switched on. canId: %d.\n",canId);
            }
            if(message->data[4] & 4) //0000 0100 (bit 2)
            {
                CD_INFO("\t-Operation Enabled. canId: %d.\n",canId);
            }
            if(message->data[4] & 8) //0000 1000 (bit 3)
            {
                CD_INFO("\t-Fault. If set, a fault condition is or was present in the drive. canId: %d.\n",canId);
            }
            if(message->data[4] & 16) //0001 0000 (bit 4)
            {
                CD_INFO("\t-Motor supply voltage is present. canId: %d.\n",canId);//true
            }
            else
            {
                CD_INFO("\t-Motor supply voltage is absent. canId: %d.\n",canId);//false
            }
            if(!(message->data[4] & 32)) //0010 0000 (bit 5), negated.
            {
                CD_INFO("\t-Performing a quick stop. canId: %d.\n",canId);
            }
            if(message->data[4] & 64) //0100 0000 (bit 6)
            {
                CD_INFO("\t-Switch on disabled. canId: %d.\n",canId);
            }
            if(message->data[4] & 128) //1000 0000 (bit 7)
            {
                CD_INFO("\t-Warning. A TML function / homing was called, while another TML function / homing is still in execution. The last call is ignored. canId: %d.\n",canId);
            }
            if(message->data[5] & 1) //(bit 8)
            {
                CD_INFO("\t-A TML function or homing is executed. Until the function or homing execution ends or is aborted, no other TML function / homing may be called. canId: %d.\n",canId);
            }
            if(message->data[5] & 2) //(bit 9)
            {
                CD_INFO("\t-Remote: drive parameters may be modified via CAN and the drive will execute the command message. canId: %d.\n",canId); // true
            }
            else
            {
                CD_INFO("\t-Remote: drive is in local mode and will not execute the command message (only TML internal)."); // false
            }
            if(message->data[5] & 4) //(bit 10)
            {
                targetReachedReady.wait();
                targetReached = true;
                targetReachedReady.post();
                CD_INFO("\t-Target reached. canId: %d.\n",canId);  // true
            }
            else
            {
                CD_INFO("\t-Target not reached. canId: %d.\n",canId);  // false (improvised, not in manual, but reasonable).
                targetReachedReady.wait();
                targetReached = false;
                targetReachedReady.post();
            }
            if(message->data[5] & 8) //(bit 11)
            {
                CD_INFO("\t-Internal Limit Active. canId: %d.\n",canId);
            }
            if(message->data[5] & 64) //(bit 14)
            {
                CD_INFO("\t-Last event set has ocurred. canId: %d.\n",canId); // true
            }
            else
            {
                CD_INFO("\t-No event set or the programmed event has not occurred yet. canId: %d.\n",canId); // false
            }
            if(message->data[5] & 128) //(bit 15)
            {
                CD_INFO("\t-Axis on. Power stage is enabled. Motor control is performed. canId: %d.\n",canId); // true
            }
            else
            {
                CD_INFO("\t-Axis off. Power stage is disabled. Motor control is not performed. canId: %d.\n",canId); // false
            }
            return true;
        }
        else if( (message->data[1]==0x00)&&(message->data[2]==0x20) )      // Manual 2000h: Motion Error Register
        {
            CD_INFO("Got SDO ack \"Motion Error Register\" from driver. %s\n",msgToStr(message).c_str());

            if(message->data[4] & 1) //0000 0001 (bit 0)
            {
                CD_INFO("\t*CAN error. Set when CAN controller is in error mode. canId: %d.\n",canId);
            }
            if(message->data[4] & 2) //0000 0010 (bit 1)
            {
                CD_INFO("\t*Short-circuit. Set when protection is triggered. canId: %d.\n",canId);
            }
            if(message->data[4] & 4) //0000 0100 (bit 2)
            {
                CD_INFO("\t*Invalid setup data. Set when the EEPROM stored setup data is not valid or not present. canId: %d.\n",canId);
            }
            if(message->data[4] & 8) //0000 1000 (bit 3)
            {
                CD_INFO("\t*Control error (position/speed error too big). Set when protection is triggered. canId: %d.\n",canId);
            }
            if(message->data[4] & 16) //0001 0000 (bit 4)
            {
                CD_INFO("\t*Communication error. Set when protection is triggered. canId: %d.\n",canId);//true
            }
            if(message->data[4] & 32) //0010 0000 (bit 5)
            {
                CD_INFO("\t*Motor position wraps around. Set when protection is triggered. canId: %d.\n",canId);
            }
            if(message->data[4] & 64) //0100 0000 (bit 6)
            {
                CD_INFO("\t*Positive limit switch active. Set when LSP input is in active state. canId: %d.\n",canId);
            }
            if(message->data[4] & 128) //1000 0000 (bit 7)
            {
                CD_INFO("\t*Negative limit switch active. Set when LSN input is in active state. canId: %d.\n",canId);
            }
            if(message->data[5] & 1) //(bit 8)
            {
                CD_INFO("\t*Over current. Set when protection is triggered. canId: %d.\n",canId);
            }
            if(message->data[5] & 2) //(bit 9)
            {
                CD_INFO("\t*I2t protection. Set when protection is triggered. canId: %d.\n",canId);
            }
            if(message->data[5] & 4) //(bit 10)
            {
                CD_INFO("\t*Over temperature motor. Set when protection is triggered. canId: %d.\n",canId);
            }
            if(message->data[5] & 8) //(bit 11)
            {
                CD_INFO("\t*Over temperature drive. Set when protection is triggered. canId: %d.\n",canId);
            }
            if(message->data[5] & 16) //(bit 12)
            {
                CD_INFO("\t*Over-voltage. Set when protection is triggered. canId: %d.\n",canId);
            }
            if(message->data[5] & 32) //(bit 13)
            {
                CD_INFO("\t*Under-voltage. Set when protection is triggered. canId: %d.\n",canId);
            }
            if(message->data[5] & 64) //(bit 14)
            {
                CD_INFO("\t*Command error. This bit is set in several situations. They can be distinguished either by the associated emergency code, or in conjunction with other bits. canId: %d.\n",canId);
            }
            if(message->data[5] & 128) //(bit 15)
            {
                CD_INFO("\t*Drive disabled due to enable input. Set when enable input is on disable state. canId: %d.\n",canId);
            }
            return true;
        }
        else if( (message->data[1]==0x02)&&(message->data[2]==0x10) )      // Manual 1002h contains "6041h Status word" plus Table 5.6
        {
            CD_INFO("Got \"manufacturer status register\" from driver. %s\n",msgToStr(message).c_str());

            if(message->data[4] & 1) //0000 0001 (bit 0)
            {
                CD_INFO("\t-Ready to switch on. canId: %d.\n",canId);
            }
            if(message->data[4] & 2) //0000 0010 (bit 1)
            {
                CD_INFO("\t-Switched on. canId: %d.\n",canId);
            }
            if(message->data[4] & 4) //0000 0100 (bit 2)
            {
                CD_INFO("\t-Operation Enabled. canId: %d.\n",canId);
            }
            if(message->data[4] & 8) //0000 1000 (bit 3)
            {
                CD_INFO("\t-Fault. If set, a fault condition is or was present in the drive. canId: %d.\n",canId);
            }
            if(message->data[4] & 16) //0001 0000 (bit 4)
            {
                CD_INFO("\t-Motor supply voltage is present. canId: %d.\n",canId);//true
            }
            else
            {
                CD_INFO("\t-Motor supply voltage is absent. canId: %d.\n",canId);//false
            }
            if(!(message->data[4] & 32)) //0010 0000 (bit 5), negated.
            {
                CD_INFO("\t-Performing a quick stop. canId: %d.\n",canId);
            }
            if(message->data[4] & 64) //0100 0000 (bit 6)
            {
                CD_INFO("\t-Switch on disabled. canId: %d.\n",canId);
            }
            if(message->data[4] & 128) //1000 0000 (bit 7)
            {
                CD_INFO("\t-Warning. A TML function / homing was called, while another TML function / homing is still in execution. The last call is ignored. canId: %d.\n",canId);
            }
            if(message->data[5] & 1) //(bit 8)
            {
                CD_INFO("\t-A TML function or homing is executed. Until the function or homing execution ends or is aborted, no other TML function / homing may be called. canId: %d.\n",canId);
            }
            if(message->data[5] & 2) //(bit 9)
            {
                CD_INFO("\t-Remote: drive parameters may be modified via CAN and the drive will execute the command message. canId: %d.\n",canId); // true
            }
            else
            {
                CD_INFO("\t-Remote: drive is in local mode and will not execute the command message (only TML internal)."); // false
            }
            if(message->data[5] & 4) //(bit 10)
            {
                CD_INFO("\t-Target reached. canId: %d.\n",canId);  // true
            }
            else
            {
                CD_INFO("\t-Target not reached. canId: %d.\n",canId);  // false (improvised, not in manual, but reasonable).
            }
            if(message->data[5] & 8) //(bit 11)
            {
                CD_INFO("\t-Internal Limit Active. canId: %d.\n",canId);
            }
            if(message->data[5] & 64) //(bit 14)
            {
                CD_INFO("\t-Last event set has ocurred. canId: %d.\n",canId); // true
            }
            else
            {
                CD_INFO("\t-No event set or the programmed event has not occurred yet. canId: %d.\n",canId); // false
            }
            if(message->data[5] & 128) //(bit 15)
            {
                CD_INFO("\t-Axis on. Power stage is enabled. Motor control is performed. canId: %d.\n",canId); // true
            }
            else
            {
                CD_INFO("\t-Axis off. Power stage is disabled. Motor control is not performed. canId: %d.\n",canId); // false
            }
            ////Much much more in Table 5.6
            if(message->data[6] & 1) //(bit 16)
            {
                CD_INFO("\t*Drive/motor initialization performed. canId: %d.\n",canId);
            }
            if(message->data[6] & 2) //(bit 17)
            {
                CD_INFO("\t*Position trigger 1 reached. canId: %d.\n",canId);
            }
            if(message->data[6] & 4) //(bit 18)
            {
                CD_INFO("\t*Position trigger 2 reached. canId: %d.\n",canId);
            }
            if(message->data[6] & 8) //(bit 19)
            {
                CD_INFO("\t*Position trigger 3 reached. canId: %d.\n",canId);
            }
            if(message->data[6] & 16) //(bit 20)
            {
                CD_INFO("\t*Position trigger 4 reached. canId: %d.\n",canId);
            }
            if(message->data[6] & 32) //(bit 21)
            {
                CD_INFO("\t*AUTORUN mode enabled. canId: %d.\n",canId);
            }
            if(message->data[6] & 64) //(bit 22)
            {
                CD_INFO("\t*Limit switch positive event / interrupt triggered. canId: %d.\n",canId);
            }
            if(message->data[6] & 128) //(bit 23)
            {
                CD_INFO("\t*Limit switch negative event / interrupt triggered. canId: %d.\n",canId);
            }
            if(message->data[7] & 1) //(bit 24)
            {
                CD_INFO("\t*Capture event/interrupt triggered. canId: %d.\n",canId);
            }
            if(message->data[7] & 2) //(bit 25)
            {
                CD_INFO("\t*Target command reached. canId: %d.\n",canId);
            }
            if(message->data[7] & 4) //(bit 26)
            {
                CD_INFO("\t*Motor I2t protection warning level reached. canId: %d.\n",canId);
            }
            if(message->data[7] & 8) //(bit 27)
            {
                CD_INFO("\t*Drive I2t protection warning level reached. canId: %d.\n",canId);
            }
            if(message->data[7] & 16) //(bit 28)
            {
                CD_INFO("\t*Gear ratio in electronic gearing mode reached. canId: %d.\n",canId);
            }
            if(message->data[7] & 64) //(bit 30)
            {
                CD_INFO("\t*Reference position in absolute electronic camming mode reached. canId: %d.\n",canId);
            }
            if(message->data[7] & 128) //(bit 31)
            {
                CD_INFO("\t*Drive/motor in fault status. canId: %d.\n",canId);
            }
            return true;
        }
        else if( (message->data[1]==0x02)&&(message->data[2]==0x20) )      // 2002h: Detailed Error Register
        {
            CD_INFO("Got SDO ack \"Detailed Error Register\" from driver. %s\n",msgToStr(message).c_str());

            if(message->data[4] & 1) //0000 0001 (bit 0)
            {
                CD_INFO("\t**The number of nested function calls exceeded the length of TML stack. Last function call was ignored. canId: %d.\n",canId);
            }
            if(message->data[4] & 2) //0000 0010 (bit 1)
            {
                CD_INFO("\t**A RET/RETI instruction was executed while no function/ISR was active. canId: %d.\n",canId);
            }
            if(message->data[4] & 4) //0000 0100 (bit 2)
            {
                CD_INFO("\t**A call to an inexistent homing routine was received. canId: %d.\n",canId);
            }
            if(message->data[4] & 8) //0000 1000 (bit 3)
            {
                CD_INFO("\t**A call to an inexistent function was received. canId: %d.\n",canId);
            }
            if(message->data[4] & 16) //0001 0000 (bit 4)
            {
                CD_INFO("\t**UPD instruction received while AXISON was executed. The UPD instruction was ingnored and it must be sent again when AXISON is completed. canId: %d.\n",canId);
            }
            if(message->data[4] & 32) //0010 0000 (bit 5)
            {
                CD_INFO("\t**Cancelable call instruction received while another cancelable function was active. canId: %d.\n",canId);
            }
            if(message->data[4] & 64) //0100 0000 (bit 6)
            {
                CD_INFO("\t**Positive software limit switch is active. canId: %d.\n",canId);
            }
            if(message->data[4] & 128) //1000 0000 (bit 7)
            {
                CD_INFO("\t**Negative software limit switch is active. canId: %d.\n",canId);
            }
            if(message->data[5] & 1) //(bit 8)
            {
                CD_INFO("\t**S-curve parameters caused and invalid profile. UPD instruction was ignored. canId: %d.\n",canId);
            }
            if(message->data[5] & 2) //(bit 9)
            {
                CD_INFO("\t**Update ignored for S-curve. canId: %d.\n",canId);
            }
            if(message->data[5] & 4) //(bit 10)
            {
                CD_INFO("\t**Encoder broken wire. canId: %d.\n",canId);
            }
            if(message->data[5] & 8) //(bit 11)
            {
                CD_INFO("\t**Motionless start failed. canId: %d.\n",canId);
            }
            if(message->data[5] & 32) //(bit 13)
            {
                CD_INFO("\t**Self check error. canId: %d.\n",canId);
            }
            return true;
        }
        else if( (message->data[1]==0x83)&&(message->data[2]==0x60) )      // Manual 8.2.3. 6083h: Profile acceleration
        {
            CD_INFO("Got SDO ack \"posmode_acc\" from driver. %s\n",msgToStr(message).c_str());
            return true;
        }
        else if( (message->data[1]==0x81)&&(message->data[2]==0x60) )      // Manual 8.2.2. 6081h: Profile velocity
        {
            CD_INFO("Got SDO ack \"posmode_speed\" from driver. %s\n",msgToStr(message).c_str());
            return true;
        }
        else if( (message->data[1]==0x7D)&&(message->data[2]==0x60) )      // Manual 607Dh: Software position limit
        {
            if (message->data[3]==0x01)
            {
                CD_INFO("Got SDO ack \"msg_position_min\" from driver. %s\n",msgToStr(message).c_str());
            }
            else if (message->data[3]==0x02)
            {
                CD_INFO("Got SDO ack \"msg_position_max\" from driver. %s\n",msgToStr(message).c_str());
            }
            else
            {
                CD_WARNING("Got SDO ack \"msg_position_????\" from driver. %s\n",msgToStr(message).c_str());
                return false;
            }
            return true;
        }
        else if( (message->data[1]==0x81)&&(message->data[2]==0x20) )      // Manual 2081h: Set/Change the actual motor position
        {
            CD_INFO("Got SDO ack \"set encoder\" from driver. %s\n",msgToStr(message).c_str());
            return true;
        }
        else if( (message->data[1]==0x02)&&(message->data[2]==0x16) )      // Manual 1602h: Receive PDO3 Mapping Parameters
        {
            CD_INFO("Got SDO ack \"RPDO3 changes\" from driver. %s\n",msgToStr(message).c_str());
            return true;
        }
        else if( (message->data[1]==0xC0)&&(message->data[2]==0x60) )      // Manual 60C0h: Interpolation sub mode select
        {
            CD_INFO("Got SDO ack \"Interpolation sub mode select.\" from driver. %s\n",msgToStr(message).c_str());
            return true;
        }
        else if( (message->data[1]==0x74)&&(message->data[2]==0x20) )      // Manual 2074h: Interpolated position buffer configuration
        {
            CD_INFO("Got SDO ack \"Interpolated position buffer configuration.\" from driver. %s\n",msgToStr(message).c_str());
            return true;
        }
        else if( (message->data[1]==0x79)&&(message->data[2]==0x20) )      // Manual 2079h: Interpolated position initial position
        {
            CD_INFO("Got SDO ack \"Interpolated position initial position.\" from driver. %s\n",msgToStr(message).c_str());
            return true;
        }
        CD_INFO("Got SDO ack from driver side: type not known. %s\n",msgToStr(message).c_str());
        return false;
    }
    else if( (message->id-canId) == 0x180 )  // ---------------------- PDO1 ----------------------
    {
        if( (message->data[0]==0x37)&&(message->data[1]==0x92) )
        {
            CD_INFO("Got PDO1 that it is observed as ack \"start position\" from driver. %s\n",msgToStr(message).c_str());
            return true;
        }
        else if( (message->data[0]==0x37)&&(message->data[1]==0x86) )
        {
            CD_INFO("Got PDO1 that it is observed when driver arrives to position target. %s\n",msgToStr(message).c_str());
            return true;
        }
        else if( (message->data[0]==0x40)&&(message->data[1]==0x02) )
        {
            CD_INFO("Got PDO1 that it is observed as TRANSITION performed upon \"start\". %s\n",msgToStr(message).c_str());
            return true;
        }
        else if( (message->data[0]==0x40)&&(message->data[1]==0x03) )
        {
            CD_INFO("Got PDO1 that it is observed as part of TRANSITION performed upon \"readyToSwitchOn\". %s\n",msgToStr(message).c_str());
            return true;
        }
        else if( (message->data[0]==0x21)&&(message->data[1]==0x02) )
        {
            CD_INFO("Got PDO1 that it is observed as part of TRANSITION performed upon \"readyToSwitchOn\". %s\n",msgToStr(message).c_str());
            return true;
        }
        else if( (message->data[0]==0x21)&&(message->data[1]==0x03) )
        {
            this->getSwitchOnReady.wait();
            this->getSwitchOn = true;
            this->getSwitchOnReady.post();
            CD_INFO("Got PDO1 that it is observed as part of TRANSITION performed upon \"switchOn\". %s\n",msgToStr(message).c_str());
            return true;
        }
        else if( (message->data[0]==0x33)&&(message->data[1]==0x83) )
        {
            this->getEnableReady.wait();
            this->getEnable = true;
            this->getEnableReady.post();
            CD_INFO("Got PDO1 that it is observed as part of TRANSITION performed upon \"enable\". %s\n",msgToStr(message).c_str());
            return true;
        }
        CD_INFO("Got PDO1 from driver side: unknown. %s\n",msgToStr(message).c_str());
        return false;
    }
    else if( (message->id-canId) == 0x280 )  // PDO2
    {
        if( (message->data[0]==0x37)&&(message->data[1]==0x92) )
        {
            CD_INFO("Got PDO2 that it is observed as ack \"start position\" from driver. %s\n",msgToStr(message).c_str());
            return true;
        }
        else if( (message->data[0]==0x37)&&(message->data[1]==0x86) )
        {
            CD_INFO("Got PDO2 that it is observed when driver arrives to position target. %s\n",msgToStr(message).c_str());
            return true;
        }
        else if( (message->data[0]==0x40)&&(message->data[1]==0x02) )
        {
            CD_INFO("Got PDO2 that it is observed as TRANSITION performed upon \"start\". %s\n",msgToStr(message).c_str());
            return true;
        }
        else if( (message->data[0]==0x40)&&(message->data[1]==0x03) )
        {
            CD_INFO("Got PDO2 that it is observed as part of TRANSITION performed upon \"readyToSwitchOn\". %s\n",msgToStr(message).c_str());
            return true;
        }
        else if( (message->data[0]==0x21)&&(message->data[1]==0x02) )
        {
            CD_INFO("Got PDO2 that it is observed as part of TRANSITION performed upon \"readyToSwitchOn\". %s\n",msgToStr(message).c_str());
            return true;
        }
        else if( (message->data[0]==0x21)&&(message->data[1]==0x03) )
        {
            this->getSwitchOnReady.wait();
            this->getSwitchOn = true;
            this->getSwitchOnReady.post();
            CD_INFO("Got PDO2 that it is observed as part of TRANSITION performed upon \"switchOn\". %s\n",msgToStr(message).c_str());
            return true;
        }
        else if( (message->data[0]==0x83)&&(message->data[1]==0x83) )
        {
            this->getEnableReady.wait();
            this->getEnable = true;
            this->getEnableReady.post();
            CD_INFO("Got PDO2 that it is observed as part of TRANSITION performed upon \"enable\". %s\n",msgToStr(message).c_str());
            return true;
        }
        CD_INFO("Got PDO2 from driver side: unknown. %s\n",msgToStr(message).c_str());
        return false;
    }
    else if( (message->id-canId) == 0x80 )  // EMERGENCY (EMCY), Table 4.2 Emergency Error Codes (p57, 73/263)
    {
        CD_ERROR("Got EMERGENCY from iPOS. %s ",msgToStr(message).c_str());
        if( (message->data[1]==0x00)&&(message->data[0]==0x00) )
        {
            CD_ERROR_NO_HEADER("Error Reset or No Error. canId: %d.\n",canId);
            return true;
        }
        else if ( (message->data[1]==0x10)&&(message->data[0]==0x00) )
        {
            CD_ERROR_NO_HEADER("Generic error. canId: %d.\n",canId);
            return true;
        }
        else if (message->data[1]==0x23)
        {
            if (message->data[0]==0x10)
            {
                CD_ERROR_NO_HEADER("Continuous over-current. canId: %d.\n",canId);
                return true;
            }
            else if (message->data[0]==0x40)
            {
                CD_ERROR_NO_HEADER("Short-circuit. canId: %d.\n",canId);
                return true;
            }
            CD_ERROR_NO_HEADER("NOT SPECIFIED IN MANUAL. canId: %d.\n",canId);
            return false;
        }
        else if (message->data[1]==0x32)
        {
            if (message->data[0]==0x10)
            {
                CD_ERROR_NO_HEADER("DC-link over-voltage. canId: %d.\n",canId);
                return true;
            }
            else if (message->data[0]==0x20)
            {
                CD_ERROR_NO_HEADER("DC-link under-voltage. canId: %d.\n",canId);
                return true;
            }
            CD_ERROR_NO_HEADER("NOT SPECIFIED IN MANUAL. canId: %d.\n",canId);
            return false;
        }
        else if ( (message->data[1]==0x42)&&(message->data[0]==0x80) )
        {
            CD_ERROR_NO_HEADER("Over temperature motor. canId: %d.\n",canId);
            return true;
        }
        else if ( (message->data[1]==0x43)&&(message->data[0]==0x10) )
        {
            CD_ERROR_NO_HEADER("Over temperature drive. canId: %d.\n",canId);
            return true;
        }
        else if (message->data[1]==0x54)
        {
            if (message->data[0]==0x41)
            {
                CD_ERROR_NO_HEADER("Driver disabled due to enable input. canId: %d.\n",canId);
                return true;
            }
            else if (message->data[0]==0x42)
            {
                CD_ERROR_NO_HEADER("Negative limit switch active. canId: %d.\n",canId);
                return true;
            }
            else if (message->data[0]==0x43)
            {
                CD_ERROR_NO_HEADER("Positive limit switch active. canId: %d.\n",canId);
                return true;
            }
            CD_ERROR_NO_HEADER("NOT SPECIFIED IN MANUAL. canId: %d.\n",canId);
            return false;
        }
        else if ( (message->data[1]==0x61)&&(message->data[0]==0x00) )
        {
            CD_ERROR_NO_HEADER("Invalid setup data. canId: %d.\n",canId);
            return true;
        }
        else if ( (message->data[1]==0x75)&&(message->data[0]==0x00) )
        {
            CD_ERROR_NO_HEADER("Communication error. canId: %d.\n",canId);
            return true;
        }
        else if (message->data[1]==0x81)
        {
            if (message->data[0]==0x10)
            {
                CD_ERROR_NO_HEADER("CAN overrun (message lost). canId: %d.\n",canId);
                return true;
            }
            else if (message->data[0]==0x30)
            {
                CD_ERROR_NO_HEADER("Life guard error or heartbeat error. canId: %d.\n",canId);
                return true;
            }
            CD_ERROR_NO_HEADER("NOT SPECIFIED IN MANUAL. canId: %d.\n",canId);
            return false;
        }
        else if ( (message->data[1]==0x83)&&(message->data[0]==0x31) )
        {
            CD_ERROR_NO_HEADER("I2t protection triggered. canId: %d.\n",canId);
            return true;
        }
        else if ( (message->data[1]==0x85)&&(message->data[0]==0x80) )
        {
            CD_ERROR_NO_HEADER("Position wraparound / Hal sensor missing. canId: %d.\n",canId);
            return true;
        }
        else if ( (message->data[1]==0x86)&&(message->data[0]==0x11) )
        {
            CD_ERROR_NO_HEADER("Control error / Following error. canId: %d.\n",canId);
            return true;
        }
        else if ( (message->data[1]==0x90)&&(message->data[0]==0x00) )
        {
            CD_ERROR_NO_HEADER("Command error canId: %d.\n",canId);
            return true;
        }
        else if (message->data[1]==0xFF)
        {
            if (message->data[0]==0x01)
            {
                CD_ERROR_NO_HEADER("Generic interpolated position mode error (PVT / PT error). canId: %d.\n",canId);
                return true;
            }
            else if (message->data[0]==0x02)
            {
                CD_ERROR_NO_HEADER("Change set acknowledge bit wrong value. canId: %d.\n",canId);
                return true;
            }
            else if (message->data[0]==0x03)
            {
                CD_ERROR_NO_HEADER("Specified homing method not available. canId: %d.\n",canId);
                return true;
            }
            else if (message->data[0]==0x04)
            {
                CD_ERROR_NO_HEADER("A wrong mode is set in object 6060h, modes_of_operation. canId: %d.\n",canId);
                return true;
            }
            else if (message->data[0]==0x05)
            {
                CD_ERROR_NO_HEADER("Specified digital I/O line not available. canId: %d.\n",canId);
                return true;
            }
            CD_ERROR_NO_HEADER("NOT SPECIFIED IN MANUAL. canId: %d.\n",canId);
            return false;
        }
        CD_ERROR_NO_HEADER("NOT SPECIFIED IN MANUAL. canId: %d.\n",canId);
        return false;
    }

    CD_ERROR("Unknown message: %s\n", msgToStr(message).c_str());

    return false;

}  //-- ends interpretMessage

// -----------------------------------------------------------------------------
