// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TechnosoftIpos.hpp"

// -----------------------------------------------------------------------------

namespace
{
    bool retrieveDrivePeakCurrent(uint32_t productCode, double *peakCurrent)
    {
        switch (productCode)
        {
            case 24300101: // iPOS2401 MX-CAN
            case 24200121: // iPOS2401 MX-CAT
                *peakCurrent = 0.9;
                break;
            case 28001001: // iPOS3602 VX-CAN
            case 28001021: // iPOS3602 VX-CAT
            case 28001101: // iPOS3602 MX-CAN
            case 28001201: // iPOS3602 BX-CAN
            case 28001501: // iPOS3602 HX-CAN
                *peakCurrent = 3.2;
                break;
            case 28002001: // iPOS3604 VX-CAN
            case 28002021: // iPOS3604 VX-CAT
            case 28002101: // iPOS3604 MX-CAN
            case 28002201: // iPOS3604 BX-CAN
            case 28002501: // iPOS3604 HX-CAN
                *peakCurrent = 10.0;
                break;
            case 27014001: // iPOS4808 VX-CAN
            case 27014101: // iPOS4808 MX-CAN
            case 27014121: // iPOS4808 MX-CAT
            case 27414101: // iPOS4808 MY-CAN (standard)
            case 27424101: // iPOS4808 MY-CAN (extended)
            case 27314111: // iPOS4808 MY-CAN-STO (standard)
            case 27324111: // iPOS4808 MY-CAN-STO (extended)
            case 27314121: // iPOS4808 MY-CAT-STO (standard)
            case 27324121: // iPOS4808 MY-CAT-STO (extended)
            case 27014201: // iPOS4808 BX-CAN
            case 27214201: // iPOS4808 BX-CAN (standard)
            case 27214701: // iPOS4808 BX-CAN (hall)
            case 27214221: // iPOS4808 BX-CAT (standard)
            case 27214721: // iPOS4808 BX-CAT (hall)
            case 27314221: // iPOS4808 BX-CAT-STO (standard)
            case 27314721: // iPOS4808 BX-CAT-STO (hall)
            case 29025201: // iPOS8010 BX-CAN
            case 29025221: // iPOS8010 BX-CAT
            case 29025202: // iPOS8010 BA-CAN
            case 29025222: // iPOS8010 BA-CAT
                *peakCurrent = 20.0;
                break;
            case 29026201: // iPOS8020 BX-CAN
            case 29026221: // iPOS8020 BX-CAT
            case 29026202: // iPOS8020 BA-CAN
            case 29026222: // iPOS8020 BA-CAT
                *peakCurrent = 40.0;
                break;
            default:
                return false;
        }

        return true;
    }
}

// -----------------------------------------------------------------------------

bool roboticslab::TechnosoftIpos::setCanBusPtr(yarp::dev::ICanBus *canDevicePtr)
{

    this->canDevicePtr = canDevicePtr;
    CD_SUCCESS("Ok pointer to CAN bus device %d.\n",canId);

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::TechnosoftIpos::setIEncodersTimedRawExternal(IEncodersTimedRaw * iEncodersTimedRaw)
{
    double v;
    this->iEncodersTimedRawExternal = iEncodersTimedRaw;

    CD_SUCCESS("Ok pointer to external encoder interface %p (%d). Updating with latest external...\n",iEncodersTimedRaw,canId);

    CD_INFO("canId(%d) wait to get external encoder value...\n",this->canId);
    while( !iEncodersTimedRawExternal->getEncoderRaw(0,&v) )  //-- loop while v is still a NaN.
    {
        //CD_INFO("Wait to get external encoder value...\n"); //\todo{activate these lines if blocking is too much}
        //Time::delay(0.2);
    }
    this->setEncoderRaw(0,v);  //-- Forces the relative encoder to this value.

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::TechnosoftIpos::initialize()
{
    uint8_t msg_deviceType[] = {0x40,0x00,0x10,0x00};

    if (!send(0x600, 4, msg_deviceType))
    {
        CD_ERROR("Could not send \"Device Type\" query. %s\n", msgToStr(0x600, 4, msg_deviceType).c_str());
        return false;
    }

    CD_SUCCESS("Sent \"Device Type\" query. %s\n", msgToStr(0x600, 4, msg_deviceType).c_str());

    uint8_t msg_supportedDriveModes[] = {0x40,0x02,0x65,0x00};

    if (!send(0x600, 4, msg_supportedDriveModes))
    {
        CD_ERROR("Could not send \"Supported drive modes\" query. %s\n", msgToStr(0x600, 4, msg_supportedDriveModes).c_str());
        return false;
    }

    CD_SUCCESS("Sent \"Supported drive modes\" query. %s\n", msgToStr(0x600, 4, msg_supportedDriveModes).c_str());

    uint8_t msg_identityObject[] = {0x40,0x18,0x10,0x00};

    msg_identityObject[3] = 0x01;

    if (!send(0x600, 4, msg_identityObject))
    {
        CD_ERROR("Could not send \"Vendor ID\" query. %s\n", msgToStr(0x600, 4, msg_identityObject).c_str());
        return false;
    }

    CD_SUCCESS("Sent \"Vendor ID\" query. %s\n", msgToStr(0x600, 4, msg_identityObject).c_str());

    msg_identityObject[3] = 0x02;

    if (!send(0x600, 4, msg_identityObject))
    {
        CD_ERROR("Could not send \"Product Code\" query. %s\n", msgToStr(0x600, 4, msg_identityObject).c_str());
        return false;
    }

    CD_SUCCESS("Sent \"Product Code\" query. %s\n", msgToStr(0x600, 4, msg_identityObject).c_str());

    int count = 0;
    const int retries = 10;
    uint32_t productCode;

    do
    {
        getProductCodeReady.wait();
        productCode = getProductCode;
        getProductCodeReady.post();

        if (productCode)
        {
            break;
        }

        count++;

        if (count == retries)
        {
            CD_ERROR("Max retries exceeded on awaiting for product code response.\n");
            return false;
        }

        yarp::os::Time::delay(DELAY);
    }
    while (true);

    if (!retrieveDrivePeakCurrent(productCode, &drivePeakCurrent))
    {
        CD_ERROR("Unhandled iPOS model %d, unable to retrieve drive peak current.\n", productCode);
        return false;
    }

    CD_SUCCESS("Retrieved drive peak current: %f A.\n", drivePeakCurrent);

    msg_identityObject[3] = 0x03;

    if (!send(0x600, 4, msg_identityObject))
    {
        CD_ERROR("Could not send \"Revision number\" query. %s\n", msgToStr(0x600, 4, msg_identityObject).c_str());
        return false;
    }

    CD_SUCCESS("Sent \"Revision number\" query. %s\n", msgToStr(0x600, 4, msg_identityObject).c_str());

    msg_identityObject[3] = 0x04;

    if (!send(0x600, 4, msg_identityObject))
    {
        CD_ERROR("Could not send \"Serial number\" query. %s\n", msgToStr(0x600, 4, msg_identityObject).c_str());
        return false;
    }

    CD_SUCCESS("Sent \"Serial number\" query. %s\n", msgToStr(0x600, 4, msg_identityObject).c_str());

    uint8_t msg_quickStopOptionCode[] = {0x2B,0x5A,0x60,0x00,0x06,0x00,0x00,0x00};

    if (!send(0x600, 8, msg_quickStopOptionCode))
    {
        CD_ERROR("Could not send \"Quick stop option code\". %s\n", msgToStr(0x600, 8, msg_quickStopOptionCode).c_str());
        return false;
    }

    CD_SUCCESS("Sent \"Quick stop option code\". %s\n", msgToStr(0x600, 8, msg_quickStopOptionCode).c_str());

    return true;
}

// -----------------------------------------------------------------------------
/** -- Start Remote Node: Used to change NMT state of one or all NMT slaves to Operational.
 PDO communication will beallowed. */

bool roboticslab::TechnosoftIpos::start()
{

    yarp::dev::CanMessage &msg_start = canOutputBuffer[0];
    msg_start.setId(0);
    msg_start.setLen(2);

    // NMT Start Remote Node (to operational, Fig 4.1)
    msg_start.getData()[0] = 0x01;
    msg_start.getData()[1] = canId;

    unsigned int sent;

    if( ! canDevicePtr->canWrite(canOutputBuffer, 1, &sent, true) || sent == 0 )
    {
        CD_ERROR("Could not send \"start\". %s\n", msgToStr(msg_start).c_str() );
        return false;
    }
    CD_SUCCESS("Sent \"start\". %s\n", msgToStr(msg_start).c_str() );

    //-- Do not force expect response as only happens upon transition.
    //-- For example, if already started, function would get stuck.

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::TechnosoftIpos::readyToSwitchOn()
{

    uint8_t msg_readyToSwitchOn[] = {0x06,0x00}; //-- readyToSwitchOn, also acts as shutdown.
    // -- send se diferencia de senRaw en que tiene un delay y adems incluye el ID (mirar funcin)
    if( ! this->send( 0x200, 2, msg_readyToSwitchOn) ) // -- 0x200 (valor critico que se pone sin saber que significa) 2 (tamano del mensaje)
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

bool roboticslab::TechnosoftIpos::switchOn()
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

bool roboticslab::TechnosoftIpos::enable()
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

bool roboticslab::TechnosoftIpos::recoverFromError()
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

bool roboticslab::TechnosoftIpos::resetNodes()
{

    // NMT Reset Node (Manual 4.1.2.3)
    yarp::dev::CanMessage &msg_resetNodes = canOutputBuffer[0];
    msg_resetNodes.setId(0);
    msg_resetNodes.setLen(2);

    // reset all nodes ([0x00] = broadcast)
    msg_resetNodes.getData()[0] = 0x81;
    msg_resetNodes.getData()[1] = 0x00;

    unsigned int sent;

    //msg_resetNode[1]=this->canId; // -- It writes canId in byte 1
    if( ! canDevicePtr->canWrite(canOutputBuffer, 1, &sent, true) || sent == 0 ) // -- 0 (hace referencia al ID. Si est en 0 es como un broadcast) 2 (tamao del mensaje)
    {
        CD_ERROR("Could not send \"reset node\". %s\n", msgToStr(msg_resetNodes).c_str() );
        return false;
    }
    CD_SUCCESS("Sent \"reset nodes\". %s\n", msgToStr(msg_resetNodes).c_str() );

    //-- Do not force expect response as only happens upon transition.
    //-- For example, if already started, function would get stuck.

    return true;
}

/** Manual: 4.1.2. Device control
 * The NMT master sets the state of the selected NMT slave to the “reset communication” sub-state.
 * In this state the drives resets their communication and enter the pre-operational state.
 */

bool roboticslab::TechnosoftIpos::resetCommunication()
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

bool roboticslab::TechnosoftIpos::resetNode(int id)
{

    yarp::dev::CanMessage &msg_resetNode = canOutputBuffer[0];
    msg_resetNode.setId(id);
    msg_resetNode.setLen(2);

    // NMT Reset Node (Manual 4.1.2.3)
    msg_resetNode.getData()[0] = 0x81;
    msg_resetNode.getData()[1] = id;

    unsigned int sent;

    if( ! canDevicePtr->canWrite(canOutputBuffer, 1, &sent, true) || sent == 0 ) // -- 0 (hace referencia al ID. Si est en 0 es como un broadcast) 2 (tamao del mensaje)
    {
        CD_ERROR("Could not send \"reset node\". %s\n", msgToStr(msg_resetNode).c_str() );
        return false;
    }
    CD_SUCCESS("Sent \"reset node\". %s\n", msgToStr(msg_resetNode).c_str() );

    //-- Do not force expect response as only happens upon transition.
    //-- For example, if already started, function would get stuck.

    return true;
}



// -----------------------------------------------------------------------------

bool roboticslab::TechnosoftIpos::interpretMessage(const yarp::dev::CanMessage & message)
{

    //--------------- Give high priority to PT, override EMGY red -------------------------

    if( (message.getData()[1]==0xFF)&&(message.getData()[2]==0x01)&&(message.getData()[4]==0x20) )
    {
        ptBuffer.wait();
        CD_WARNING("pt buffer full! canId: %d.\n",canId);
        return true;
    }
    else if( (message.getData()[1]==0xFF)&&(message.getData()[2]==0x01)&&(message.getData()[4]==0x00) )
    {
        ptBuffer.post();
        CD_WARNING("pt buffer empty. canId: %d.\n",canId);
        return true;
    }
    else if( (message.getData()[1]==0xFF)&&(message.getData()[2]==0x01)&&(message.getData()[3]==0x08) )
    {
        CD_WARNING("pt buffer message (don't know what it means). canId: %d.\n",canId);
        return true;
    }
    else if( (message.getData()[0]==0x37)&&(message.getData()[1]==0x96) )
    {
        CD_WARNING("pt movement ended. canId: %d (via %X).\n",canId,message.getId()-canId);
        ptMovementDone = true;
        return true;
    }
    else if (message.getData()[0] == 0x01 && message.getData()[1] == 0xFF)
    {
        CD_INFO("PVT control message. canId: %d.\n",canId);
        return true;
    }
    else if( (message.getId()-canId) == 0x580 )  // -------------- SDO ----------------------
    {
        if( (message.getData()[1]==0x64) && (message.getData()[2]==0x60) )  // Manual 6064h
        {
            //-- Commenting encoder value (response to petition) as way too verbose, happens all the time.
            //CD_INFO("Got encoder value (response to petition). %s\n",msgToStr(message).c_str());
            int got;
            memcpy(&got, message.getData()+4,4);
            lastEncoderRead.update(got / ((encoderPulses / 360.0) * this->tr));
            return true;
        }
        else if( (message.getData()[1]==0x7E) && (message.getData()[2]==0x20) )     // Manual 207Eh
        {
            //-- Commenting current value (response to petition) as way too verbose, happens all the time.
            //CD_INFO("Got current value (response to petition). %s\n",msgToStr(message).c_str());
            int16_t got;
            memcpy(&got, message.getData()+4,2);
            getCurrentReady.wait();
            getCurrent = got * 2.0 * drivePeakCurrent / 65520.0;
            getCurrentReady.post();
            return true;
        }
        else if( (message.getData()[1]==0x7A)&&(message.getData()[2]==0x60) )      // Manual 607Ah
        {
            CD_INFO("Got SDO ack \"position target\" from driver. %s\n",msgToStr(message).c_str());
            return true;
        }
        else if( (message.getData()[1]==0x60)&&(message.getData()[2]==0x60) )      // Manual 6060h should behave like 6061h, but ack always says mode 0.
        {
            CD_INFO("Got SDO ack \"modes of operation\" from driver. %s\n",msgToStr(message).c_str());
            return true;
        }
        else if( (message.getData()[1]==0x61)&&(message.getData()[2]==0x60) )      // Manual 6060h/6061h
        {
            CD_INFO("Got SDO \"modes of operation display\" from driver. %s\n",msgToStr(message).c_str());
            int got;
            memcpy(&got, message.getData()+4,4);
            if(251==got)  // -5
            {
                CD_INFO("\t-iPOS specific: External Reference Torque Mode. canId: %d.\n",canId);
                getModeReady.wait();
                getMode = modeCurrentTorque == VOCAB_CM_TORQUE ? VOCAB_CM_TORQUE : VOCAB_CM_CURRENT;
                getModeReady.post();
            }
            else if(252==got)  // -4
            {
                CD_INFO("\t-iPOS specific: External Reference Speed Mode. canId: %d.\n",canId);
                getModeReady.wait();
                getMode = 0;
                getModeReady.post();
            }
            else if(253==got)  // -3
            {
                CD_INFO("\t-iPOS specific: External Reference Position Mode. canId: %d.\n",canId);
                getModeReady.wait();
                getMode = VOCAB_CM_POSITION_DIRECT;
                getModeReady.post();
            }
            else if(254==got)  // -2
            {
                CD_INFO("\t-iPOS specific: Electronic Camming Position Mode. canId: %d.\n",canId);
                getModeReady.wait();
                getMode = 0;
                getModeReady.post();
            }
            else if(255==got)  // -1
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
                getMode = 0;
                getModeReady.post();
            }
            else if(8==got)
            {
                CD_INFO("\t-Cyclic Synchronous Position Mode. canId: %d.\n",canId);
                getModeReady.wait();
                getMode = 0;
                getModeReady.post();
            }
            else
            {
                CD_WARNING("\t-Mode \"%d\" not specified in manual, may be in Fault or not enabled yet. canId(%d).\n",got,(message.getId() & 0x7F));
                getModeReady.wait();
                getMode = VOCAB_FAILED;
                getModeReady.post();
                return true;
            }
            return true;
        }
        else if( (message.getData()[1]==0x41)&&(message.getData()[2]==0x60) )      // Manual 6041h: Status word; Table 5.4 Bit Assignment in Status Word (also see 5.5)
        {
            CD_INFO("Got \"status word\" from driver. %s\n",msgToStr(message).c_str());

            if(message.getData()[4] & 1) //0000 0001 (bit 0)
            {
                CD_INFO("\t-Ready to switch on. canId: %d.\n",canId);
            }
            if(message.getData()[4] & 2) //0000 0010 (bit 1)
            {
                CD_INFO("\t-Switched on. canId: %d.\n",canId);
            }
            if(message.getData()[4] & 4) //0000 0100 (bit 2)
            {
                CD_INFO("\t-Operation Enabled. canId: %d.\n",canId);
            }
            if(message.getData()[4] & 8) //0000 1000 (bit 3)
            {
                CD_INFO("\t-Fault. If set, a fault condition is or was present in the drive. canId: %d.\n",canId);
            }
            if(message.getData()[4] & 16) //0001 0000 (bit 4)
            {
                CD_INFO("\t-Motor supply voltage is present. canId: %d.\n",canId);//true
            }
            else
            {
                CD_INFO("\t-Motor supply voltage is absent. canId: %d.\n",canId);//false
            }
            if(!(message.getData()[4] & 32)) //0010 0000 (bit 5), negated.
            {
                CD_INFO("\t-Performing a quick stop. canId: %d.\n",canId);
            }
            if(message.getData()[4] & 64) //0100 0000 (bit 6)
            {
                CD_INFO("\t-Switch on disabled. canId: %d.\n",canId);
            }
            if(message.getData()[4] & 128) //1000 0000 (bit 7)
            {
                CD_INFO("\t-Warning. A TML function / homing was called, while another TML function / homing is still in execution. The last call is ignored. canId: %d.\n",canId);
            }
            if(message.getData()[5] & 1) //(bit 8)
            {
                CD_INFO("\t-A TML function or homing is executed. Until the function or homing execution ends or is aborted, no other TML function / homing may be called. canId: %d.\n",canId);
            }
            if(message.getData()[5] & 2) //(bit 9)
            {
                CD_INFO("\t-Remote: drive parameters may be modified via CAN and the drive will execute the command message. canId: %d.\n",canId); // true
            }
            else
            {
                CD_INFO("\t-Remote: drive is in local mode and will not execute the command message (only TML internal)."); // false
            }
            if(message.getData()[5] & 4) //(bit 10)
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
            if(message.getData()[5] & 8) //(bit 11)
            {
                CD_INFO("\t-Internal Limit Active. canId: %d.\n",canId);
            }
            if(message.getData()[5] & 64) //(bit 14)
            {
                CD_INFO("\t-Last event set has ocurred. canId: %d.\n",canId); // true
            }
            else
            {
                CD_INFO("\t-No event set or the programmed event has not occurred yet. canId: %d.\n",canId); // false
            }
            if(message.getData()[5] & 128) //(bit 15)
            {
                CD_INFO("\t-Axis on. Power stage is enabled. Motor control is performed. canId: %d.\n",canId); // true
            }
            else
            {
                CD_INFO("\t-Axis off. Power stage is disabled. Motor control is not performed. canId: %d.\n",canId); // false
            }
            return true;
        }
        else if( (message.getData()[1]==0x00)&&(message.getData()[2]==0x20) )      // Manual 2000h: Motion Error Register
        {
            CD_INFO("Got SDO ack \"Motion Error Register\" from driver. %s\n",msgToStr(message).c_str());

            if(message.getData()[4] & 1) //0000 0001 (bit 0)
            {
                CD_INFO("\t*CAN error. Set when CAN controller is in error mode. canId: %d.\n",canId);
            }
            if(message.getData()[4] & 2) //0000 0010 (bit 1)
            {
                CD_INFO("\t*Short-circuit. Set when protection is triggered. canId: %d.\n",canId);
            }
            if(message.getData()[4] & 4) //0000 0100 (bit 2)
            {
                CD_INFO("\t*Invalid setup data. Set when the EEPROM stored setup data is not valid or not present. canId: %d.\n",canId);
            }
            if(message.getData()[4] & 8) //0000 1000 (bit 3)
            {
                CD_INFO("\t*Control error (position/speed error too big). Set when protection is triggered. canId: %d.\n",canId);
            }
            if(message.getData()[4] & 16) //0001 0000 (bit 4)
            {
                CD_INFO("\t*Communication error. Set when protection is triggered. canId: %d.\n",canId);//true
            }
            if(message.getData()[4] & 32) //0010 0000 (bit 5)
            {
                CD_INFO("\t*Motor position wraps around. Set when protection is triggered. canId: %d.\n",canId);
            }
            if(message.getData()[4] & 64) //0100 0000 (bit 6)
            {
                CD_INFO("\t*Positive limit switch active. Set when LSP input is in active state. canId: %d.\n",canId);
            }
            if(message.getData()[4] & 128) //1000 0000 (bit 7)
            {
                CD_INFO("\t*Negative limit switch active. Set when LSN input is in active state. canId: %d.\n",canId);
            }
            if(message.getData()[5] & 1) //(bit 8)
            {
                CD_INFO("\t*Over current. Set when protection is triggered. canId: %d.\n",canId);
            }
            if(message.getData()[5] & 2) //(bit 9)
            {
                CD_INFO("\t*I2t protection. Set when protection is triggered. canId: %d.\n",canId);
            }
            if(message.getData()[5] & 4) //(bit 10)
            {
                CD_INFO("\t*Over temperature motor. Set when protection is triggered. canId: %d.\n",canId);
            }
            if(message.getData()[5] & 8) //(bit 11)
            {
                CD_INFO("\t*Over temperature drive. Set when protection is triggered. canId: %d.\n",canId);
            }
            if(message.getData()[5] & 16) //(bit 12)
            {
                CD_INFO("\t*Over-voltage. Set when protection is triggered. canId: %d.\n",canId);
            }
            if(message.getData()[5] & 32) //(bit 13)
            {
                CD_INFO("\t*Under-voltage. Set when protection is triggered. canId: %d.\n",canId);
            }
            if(message.getData()[5] & 64) //(bit 14)
            {
                CD_INFO("\t*Command error. This bit is set in several situations. They can be distinguished either by the associated emergency code, or in conjunction with other bits. canId: %d.\n",canId);
            }
            if(message.getData()[5] & 128) //(bit 15)
            {
                CD_INFO("\t*Drive disabled due to enable input. Set when enable input is on disable state. canId: %d.\n",canId);
            }
            return true;
        }
        else if( (message.getData()[1]==0x02)&&(message.getData()[2]==0x10) )      // Manual 1002h contains "6041h Status word" plus Table 5.6
        {
            CD_INFO("Got \"manufacturer status register\" from driver. %s\n",msgToStr(message).c_str());

            if(message.getData()[4] & 1) //0000 0001 (bit 0)
            {
                CD_INFO("\t-Ready to switch on. canId: %d.\n",canId);
            }
            if(message.getData()[4] & 2) //0000 0010 (bit 1)
            {
                CD_INFO("\t-Switched on. canId: %d.\n",canId);
            }
            if(message.getData()[4] & 4) //0000 0100 (bit 2)
            {
                CD_INFO("\t-Operation Enabled. canId: %d.\n",canId);
            }
            if(message.getData()[4] & 8) //0000 1000 (bit 3)
            {
                CD_INFO("\t-Fault. If set, a fault condition is or was present in the drive. canId: %d.\n",canId);
            }
            if(message.getData()[4] & 16) //0001 0000 (bit 4)
            {
                CD_INFO("\t-Motor supply voltage is present. canId: %d.\n",canId);//true
            }
            else
            {
                CD_INFO("\t-Motor supply voltage is absent. canId: %d.\n",canId);//false
            }
            if(!(message.getData()[4] & 32)) //0010 0000 (bit 5), negated.
            {
                CD_INFO("\t-Performing a quick stop. canId: %d.\n",canId);
            }
            if(message.getData()[4] & 64) //0100 0000 (bit 6)
            {
                CD_INFO("\t-Switch on disabled. canId: %d.\n",canId);
            }
            if(message.getData()[4] & 128) //1000 0000 (bit 7)
            {
                CD_INFO("\t-Warning. A TML function / homing was called, while another TML function / homing is still in execution. The last call is ignored. canId: %d.\n",canId);
            }
            if(message.getData()[5] & 1) //(bit 8)
            {
                CD_INFO("\t-A TML function or homing is executed. Until the function or homing execution ends or is aborted, no other TML function / homing may be called. canId: %d.\n",canId);
            }
            if(message.getData()[5] & 2) //(bit 9)
            {
                CD_INFO("\t-Remote: drive parameters may be modified via CAN and the drive will execute the command message. canId: %d.\n",canId); // true
            }
            else
            {
                CD_INFO("\t-Remote: drive is in local mode and will not execute the command message (only TML internal)."); // false
            }
            if(message.getData()[5] & 4) //(bit 10)
            {
                CD_INFO("\t-Target reached. canId: %d.\n",canId);  // true
            }
            else
            {
                CD_INFO("\t-Target not reached. canId: %d.\n",canId);  // false (improvised, not in manual, but reasonable).
            }
            if(message.getData()[5] & 8) //(bit 11)
            {
                CD_INFO("\t-Internal Limit Active. canId: %d.\n",canId);
            }
            if(message.getData()[5] & 64) //(bit 14)
            {
                CD_INFO("\t-Last event set has ocurred. canId: %d.\n",canId); // true
            }
            else
            {
                CD_INFO("\t-No event set or the programmed event has not occurred yet. canId: %d.\n",canId); // false
            }
            if(message.getData()[5] & 128) //(bit 15)
            {
                CD_INFO("\t-Axis on. Power stage is enabled. Motor control is performed. canId: %d.\n",canId); // true
            }
            else
            {
                CD_INFO("\t-Axis off. Power stage is disabled. Motor control is not performed. canId: %d.\n",canId); // false
            }
            ////Much much more in Table 5.6
            if(message.getData()[6] & 1) //(bit 16)
            {
                CD_INFO("\t*Drive/motor initialization performed. canId: %d.\n",canId);
            }
            if(message.getData()[6] & 2) //(bit 17)
            {
                CD_INFO("\t*Position trigger 1 reached. canId: %d.\n",canId);
            }
            if(message.getData()[6] & 4) //(bit 18)
            {
                CD_INFO("\t*Position trigger 2 reached. canId: %d.\n",canId);
            }
            if(message.getData()[6] & 8) //(bit 19)
            {
                CD_INFO("\t*Position trigger 3 reached. canId: %d.\n",canId);
            }
            if(message.getData()[6] & 16) //(bit 20)
            {
                CD_INFO("\t*Position trigger 4 reached. canId: %d.\n",canId);
            }
            if(message.getData()[6] & 32) //(bit 21)
            {
                CD_INFO("\t*AUTORUN mode enabled. canId: %d.\n",canId);
            }
            if(message.getData()[6] & 64) //(bit 22)
            {
                CD_INFO("\t*Limit switch positive event / interrupt triggered. canId: %d.\n",canId);
            }
            if(message.getData()[6] & 128) //(bit 23)
            {
                CD_INFO("\t*Limit switch negative event / interrupt triggered. canId: %d.\n",canId);
            }
            if(message.getData()[7] & 1) //(bit 24)
            {
                CD_INFO("\t*Capture event/interrupt triggered. canId: %d.\n",canId);
            }
            if(message.getData()[7] & 2) //(bit 25)
            {
                CD_INFO("\t*Target command reached. canId: %d.\n",canId);
            }
            if(message.getData()[7] & 4) //(bit 26)
            {
                CD_INFO("\t*Motor I2t protection warning level reached. canId: %d.\n",canId);
            }
            if(message.getData()[7] & 8) //(bit 27)
            {
                CD_INFO("\t*Drive I2t protection warning level reached. canId: %d.\n",canId);
            }
            if(message.getData()[7] & 16) //(bit 28)
            {
                CD_INFO("\t*Gear ratio in electronic gearing mode reached. canId: %d.\n",canId);
            }
            if(message.getData()[7] & 64) //(bit 30)
            {
                CD_INFO("\t*Reference position in absolute electronic camming mode reached. canId: %d.\n",canId);
            }
            if(message.getData()[7] & 128) //(bit 31)
            {
                CD_INFO("\t*Drive/motor in fault status. canId: %d.\n",canId);
            }
            return true;
        }
        else if( (message.getData()[1]==0x02)&&(message.getData()[2]==0x20) )      // 2002h: Detailed Error Register
        {
            CD_INFO("Got SDO ack \"Detailed Error Register\" from driver. %s\n",msgToStr(message).c_str());

            if(message.getData()[4] & 1) //0000 0001 (bit 0)
            {
                CD_INFO("\t**The number of nested function calls exceeded the length of TML stack. Last function call was ignored. canId: %d.\n",canId);
            }
            if(message.getData()[4] & 2) //0000 0010 (bit 1)
            {
                CD_INFO("\t**A RET/RETI instruction was executed while no function/ISR was active. canId: %d.\n",canId);
            }
            if(message.getData()[4] & 4) //0000 0100 (bit 2)
            {
                CD_INFO("\t**A call to an inexistent homing routine was received. canId: %d.\n",canId);
            }
            if(message.getData()[4] & 8) //0000 1000 (bit 3)
            {
                CD_INFO("\t**A call to an inexistent function was received. canId: %d.\n",canId);
            }
            if(message.getData()[4] & 16) //0001 0000 (bit 4)
            {
                CD_INFO("\t**UPD instruction received while AXISON was executed. The UPD instruction was ingnored and it must be sent again when AXISON is completed. canId: %d.\n",canId);
            }
            if(message.getData()[4] & 32) //0010 0000 (bit 5)
            {
                CD_INFO("\t**Cancelable call instruction received while another cancelable function was active. canId: %d.\n",canId);
            }
            if(message.getData()[4] & 64) //0100 0000 (bit 6)
            {
                CD_INFO("\t**Positive software limit switch is active. canId: %d.\n",canId);
            }
            if(message.getData()[4] & 128) //1000 0000 (bit 7)
            {
                CD_INFO("\t**Negative software limit switch is active. canId: %d.\n",canId);
            }
            if(message.getData()[5] & 1) //(bit 8)
            {
                CD_INFO("\t**S-curve parameters caused and invalid profile. UPD instruction was ignored. canId: %d.\n",canId);
            }
            if(message.getData()[5] & 2) //(bit 9)
            {
                CD_INFO("\t**Update ignored for S-curve. canId: %d.\n",canId);
            }
            if(message.getData()[5] & 4) //(bit 10)
            {
                CD_INFO("\t**Encoder broken wire. canId: %d.\n",canId);
            }
            if(message.getData()[5] & 8) //(bit 11)
            {
                CD_INFO("\t**Motionless start failed. canId: %d.\n",canId);
            }
            if(message.getData()[5] & 32) //(bit 13)
            {
                CD_INFO("\t**Self check error. canId: %d.\n",canId);
            }
            return true;
        }
        else if( (message.getData()[1]==0x83)&&(message.getData()[2]==0x60) )      // Manual 8.2.3. 6083h: Profile acceleration
        {
            if (message.getData()[0]==0x60)      // SDO segment upload/acknowledge
            {
                CD_INFO("Got SDO ack \"posmode_acc\" from driver. %s\n",msgToStr(message).c_str());
            }
            else
            {
                uint16_t gotInteger;
                uint16_t gotFractional;
                memcpy(&gotFractional, message.getData() + 4, 2);
                memcpy(&gotInteger, message.getData() + 6, 2);
                double val = decodeFixedPoint(gotInteger, gotFractional);
                refAccelSemaphore.wait();
                refAcceleration = val / (std::abs(tr) * (encoderPulses / 360.0) * 0.000001);
                refAccelSemaphore.post();
                CD_INFO("Got SDO \"posmode_acc\" response from driver. %s\n",msgToStr(message).c_str());
            }
            return true;
        }
        else if( (message.getData()[1]==0x81)&&(message.getData()[2]==0x60) )      // Manual 8.2.2. 6081h: Profile velocity
        {
            if (message.getData()[0]==0x60)      // SDO segment upload/acknowledge
            {
                CD_INFO("Got SDO ack \"posmode_speed\" from driver. %s\n",msgToStr(message).c_str());
            }
            else      // Query
            {
                uint16_t gotInteger;
                uint16_t gotFractional;
                memcpy(&gotFractional, message.getData() + 4, 2);
                memcpy(&gotInteger, message.getData() + 6, 2);
                double val = decodeFixedPoint(gotInteger, gotFractional);
                refSpeedSemaphore.wait();
                refSpeed = val / (std::abs(tr) * (encoderPulses / 360.0) * 0.001);
                refSpeedSemaphore.post();
                CD_INFO("Got SDO \"posmode_speed\" response from driver. %s\n",msgToStr(message).c_str());
            }
            return true;
        }
        else if( (message.getData()[1]==0x7D)&&(message.getData()[2]==0x60) )      // Manual 607Dh: Software position limit
        {
            if (message.getData()[3]==0x01)
            {
                CD_INFO("Got SDO ack \"msg_position_min\" from driver. %s\n",msgToStr(message).c_str());
            }
            else if (message.getData()[3]==0x02)
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
        else if( (message.getData()[1]==0x81)&&(message.getData()[2]==0x20) )      // Manual 2081h: Set/Change the actual motor position
        {
            CD_INFO("Got SDO ack \"set encoder\" from driver. %s\n",msgToStr(message).c_str());
            return true;
        }
        else if( (message.getData()[1]==0x02)&&(message.getData()[2]==0x16) )      // Manual 1602h: Receive PDO3 Mapping Parameters
        {
            CD_INFO("Got SDO ack \"RPDO3 changes\" from driver. %s\n",msgToStr(message).c_str());
            return true;
        }
        else if( (message.getData()[1]==0xC0)&&(message.getData()[2]==0x60) )      // Manual 60C0h: Interpolation sub mode select
        {
            CD_INFO("Got SDO ack \"Interpolation sub mode select.\" from driver. %s\n",msgToStr(message).c_str());
            return true;
        }
        else if( (message.getData()[1]==0xC1)&&(message.getData()[2]==0x60) )      // Manual 60C1h: Interpolation data record
        {
            CD_INFO("Got SDO ack \"Interpolation data record.\" from driver. %s\n",msgToStr(message).c_str());
            return true;
        }
        else if( (message.getData()[1]==0x72)&&(message.getData()[2]==0x20) )      // Manual 6072h: Interpolated position mode status
        {
            CD_INFO("Got SDO ack \"Interpolated position mode status.\" from driver. %s\n",msgToStr(message).c_str());
            return true;
        }
        else if( (message.getData()[1]==0x73)&&(message.getData()[2]==0x20) )      // Manual 2073h: Interpolated position buffer length
        {
            CD_INFO("Got SDO ack \"Interpolated position buffer length.\" from driver. %s\n",msgToStr(message).c_str());
            return true;
        }
        else if( (message.getData()[1]==0x74)&&(message.getData()[2]==0x20) )      // Manual 2074h: Interpolated position buffer configuration
        {
            CD_INFO("Got SDO ack \"Interpolated position buffer configuration.\" from driver. %s\n",msgToStr(message).c_str());
            return true;
        }
        else if( (message.getData()[1]==0x7A)&&(message.getData()[2]==0x20) )      // Manual 207Ah: Interpolated position 1 st order time.
        {
            CD_INFO("Got SDO ack \"Interpolated position 1 st order time.\" from driver. %s\n",msgToStr(message).c_str());
            return true;
        }
        else if( (message.getData()[1]==0x74)&&(message.getData()[2]==0x20) )      // Manual 2074h: Interpolated position buffer configuration
        {
            CD_INFO("Got SDO ack \"Interpolated position buffer configuration.\" from driver. %s\n",msgToStr(message).c_str());
            return true;
        }
        else if( (message.getData()[1]==0x79)&&(message.getData()[2]==0x20) )      // Manual 2079h: Interpolated position initial position
        {
            CD_INFO("Got SDO ack \"Interpolated position initial position.\" from driver. %s\n",msgToStr(message).c_str());
            return true;
        }
        else if( (message.getData()[1]==0xFF)&&(message.getData()[2]==0x60) )      // Manual 60FFh: Target velocity
        {
            if (message.getData()[0]==0x60)      // SDO segment upload/acknowledge
            {
                CD_INFO("Got SDO ack \"Target velocity.\" from driver. %s\n",msgToStr(message).c_str());
            }
            else
            {
                int16_t gotInteger;
                uint16_t gotFractional;
                memcpy(&gotFractional, message.getData() + 4, 2);
                memcpy(&gotInteger, message.getData() + 6, 2);
                double val = decodeFixedPoint(gotInteger, gotFractional);
                refVelocitySemaphore.wait();
                refVelocity = val / (tr * (encoderPulses / 360.0) * 0.001);
                refVelocitySemaphore.post();
                CD_INFO("Got SDO \"Target velocity\" response from driver. %s\n",msgToStr(message).c_str());
            }
        }
        else if( (message.getData()[1]==0x5A)&&(message.getData()[2]==0x60) )      // Manual 605Ah: Quick stop option code
        {
            CD_INFO("Got SDO ack \"Quick stop option code.\" from driver. %s\n",msgToStr(message).c_str());
            return true;
        }
        else if( (message.getData()[1]==0x1C)&&(message.getData()[2]==0x20) )      // Manual 201Ch: External On-Line Reference
        {
            CD_INFO("Got SDO ack \"External On-Line Reference.\" from driver. %s\n",msgToStr(message).c_str());
            return true;
        }
        else if( (message.getData()[1]==0x1D)&&(message.getData()[2]==0x20) )      // Manual 201Dh: External Reference Type
        {
            CD_INFO("Got SDO ack \"External Reference Type.\" from driver. %s\n",msgToStr(message).c_str());
        }
        else if( (message.getData()[1]==0x00)&&(message.getData()[2]==0x10) )      // Manual 1000h: Device Type
        {
            uint16_t ciaStandard;
            memcpy(&ciaStandard, message.getData() + 4, 2);
            CD_INFO("Got \"Device Type\" from driver. %s CiA standard %d.\n",msgToStr(message).c_str(), ciaStandard);
            return true;
        }
        else if( (message.getData()[1]==0x02)&&(message.getData()[2]==0x65) )      // Manual 6502h: Supported drive modes
        {
            CD_INFO("Got \"Supported drive modes\" from driver. %s\n",msgToStr(message).c_str());

            if(message.getData()[4] & 1) // (bit 0)
            {
                CD_INFO("\t*profiled position (pp)\n");
            }
            if(message.getData()[4] & 2) // (bit 1)
            {
                CD_INFO("\t*velocity (vl)\n");
            }
            if(message.getData()[4] & 4) // (bit 2)
            {
                CD_INFO("\t*profiled velocity (pv)\n");
            }
            if(message.getData()[4] & 8) // (bit 3)
            {
                CD_INFO("\t*profiled torque (tq)\n");
            }
            if(message.getData()[4] & 32) // (bit 5)
            {
                CD_INFO("\t*homing (hm)\n");
            }
            if(message.getData()[4] & 64) // (bit 6)
            {
                CD_INFO("\t*interpolated position (ip)\n");
            }
            if(message.getData()[4] & 128) // (bit 7)
            {
                CD_INFO("\t*cyclic synchronous position\n");
            }
            if(message.getData()[5] & 1) // (bit 8)
            {
                CD_INFO("\t*cyclic synchronous velocity\n");
            }
            if(message.getData()[5] & 2) // (bit 9)
            {
                CD_INFO("\t*cyclic synchronous torque\n");
            }
            if(message.getData()[6] & 1) // (bit 16)
            {
                CD_INFO("\t*electronic camming position (manufacturer specific)\n");
            }
            if(message.getData()[6] & 2) // (bit 17)
            {
                CD_INFO("\t*electronic gearing position (manufacturer specific)\n");
            }
            if(message.getData()[6] & 4) // (bit 18)
            {
                CD_INFO("\t*external reference position (manufacturer specific)\n");
            }
            if(message.getData()[6] & 8) // (bit 19)
            {
                CD_INFO("\t*external reference speed (manufacturer specific)\n");
            }
            if(message.getData()[6] & 16) // (bit 20)
            {
                CD_INFO("\t*external reference torque (manufacturer specific)\n");
            }

            return true;
        }
        else if( (message.getData()[1]==0x18)&&(message.getData()[2]==0x10) )      // Manual 1018h: Identity Object
        {
            if( message.getData()[3]==0x01 )  // Vendor ID
            {
                CD_INFO("Got \"Vendor ID\" from driver. %s\n",msgToStr(message).c_str());
            }
            else if( message.getData()[3]==0x02 )  // Product Code
            {
                uint32_t code;
                memcpy(&code, message.getData() + 4, 4);
                CD_INFO("Got \"Product Code\" from driver. %s P%03d.%03d.E%03d.\n",msgToStr(message).c_str(),
                        code / 1000000, (code / 1000) % 1000, code % 1000);
                getProductCodeReady.wait();
                getProductCode = code;
                getProductCodeReady.post();
            }
            else if( message.getData()[3]==0x03 )  // Revision number
            {
                CD_INFO("Got \"Revision number\" from driver. %s %c%c%c%c.\n",msgToStr(message).c_str(),
                        message.getData()[7], message.getData()[6], message.getData()[6], message.getData()[4]);
            }
            else if( message.getData()[3]==0x04 )  // Serial number
            {
                CD_INFO("Got \"Serial number\" from driver. %s %c%c%02x%02x.\n",msgToStr(message).c_str(),
                        message.getData()[7], message.getData()[6], message.getData()[5], message.getData()[4]);
            }

            return true;
        }
        else if( (message.getData()[1]==0x7F)&&(message.getData()[2]==0x20) )      // Manual 207Fh: Current limit
        {
            if (message.getData()[0]==0x60)      // SDO segment upload/acknowledge
            {
                CD_INFO("Got SDO ack \"Current limit\" from driver. %s\n",msgToStr(message).c_str());
            }
            else
            {
                uint16_t got;
                memcpy(&got, message.getData() + 4, 2);
                getCurrentLimitReady.wait();
                getCurrentLimit = 2 * drivePeakCurrent * (32767 - got) / 65520;
                getCurrentLimitReady.post();
                CD_INFO("Got SDO \"Current limit.\" response from driver. %s\n",msgToStr(message).c_str());
            }
        }
        CD_INFO("Got SDO ack from driver side: type not known. %s\n",msgToStr(message).c_str());
        return false;
    }
    else if( (message.getId()-canId) == 0x180 )  // ---------------------- PDO1 ----------------------
    {
        if( (message.getData()[0]==0x37)&&(message.getData()[1]==0x92) )
        {
            CD_INFO("Got PDO1 that it is observed as ack \"start position\" from driver. %s\n",msgToStr(message).c_str());
            return true;
        }
        else if( (message.getData()[0]==0x37)&&(message.getData()[1]==0x86) )
        {
            CD_INFO("Got PDO1 that it is observed when driver arrives to position target. %s\n",msgToStr(message).c_str());
            return true;
        }
        else if( (message.getData()[0]==0x40)&&(message.getData()[1]==0x02) )
        {
            CD_INFO("Got PDO1 that it is observed as TRANSITION performed upon \"start\". %s\n",msgToStr(message).c_str());
            return true;
        }
        else if( (message.getData()[0]==0x40)&&(message.getData()[1]==0x03) )
        {
            CD_INFO("Got PDO1 that it is observed as part of TRANSITION performed upon \"readyToSwitchOn\". %s\n",msgToStr(message).c_str());
            return true;
        }
        else if( (message.getData()[0]==0x21)&&(message.getData()[1]==0x02) )
        {
            CD_INFO("Got PDO1 that it is observed as part of TRANSITION performed upon \"readyToSwitchOn\". %s\n",msgToStr(message).c_str());
            return true;
        }
        else if( (message.getData()[0]==0x21)&&(message.getData()[1]==0x03) )
        {
            this->getSwitchOnReady.wait();
            this->getSwitchOn = true;
            this->getSwitchOnReady.post();
            CD_INFO("Got PDO1 that it is observed as part of TRANSITION performed upon \"switchOn\". %s\n",msgToStr(message).c_str());
            return true;
        }
        else if( (message.getData()[0]==0x33)&&(message.getData()[1]==0x83) )
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
    else if( (message.getId()-canId) == 0x280 )  // PDO2
    {
        if( (message.getData()[0]==0x37)&&(message.getData()[1]==0x92) )
        {
            CD_INFO("Got PDO2 that it is observed as ack \"start position\" from driver. %s\n",msgToStr(message).c_str());
            return true;
        }
        else if( (message.getData()[0]==0x37)&&(message.getData()[1]==0x86) )
        {
            CD_INFO("Got PDO2 that it is observed when driver arrives to position target. %s\n",msgToStr(message).c_str());
            return true;
        }
        else if( (message.getData()[0]==0x40)&&(message.getData()[1]==0x02) )
        {
            CD_INFO("Got PDO2 that it is observed as TRANSITION performed upon \"start\". %s\n",msgToStr(message).c_str());
            return true;
        }
        else if( (message.getData()[0]==0x40)&&(message.getData()[1]==0x03) )
        {
            CD_INFO("Got PDO2 that it is observed as part of TRANSITION performed upon \"readyToSwitchOn\". %s\n",msgToStr(message).c_str());
            return true;
        }
        else if( (message.getData()[0]==0x21)&&(message.getData()[1]==0x02) )
        {
            CD_INFO("Got PDO2 that it is observed as part of TRANSITION performed upon \"readyToSwitchOn\". %s\n",msgToStr(message).c_str());
            return true;
        }
        else if( (message.getData()[0]==0x21)&&(message.getData()[1]==0x03) )
        {
            this->getSwitchOnReady.wait();
            this->getSwitchOn = true;
            this->getSwitchOnReady.post();
            CD_INFO("Got PDO2 that it is observed as part of TRANSITION performed upon \"switchOn\". %s\n",msgToStr(message).c_str());
            return true;
        }
        else if( (message.getData()[0]==0x83)&&(message.getData()[1]==0x83) )
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
    else if( (message.getId()-canId) == 0x80 )  // EMERGENCY (EMCY), Table 4.2 Emergency Error Codes (p57, 73/263)
    {
        CD_ERROR("Got EMERGENCY from iPOS. %s ",msgToStr(message).c_str());
        if( (message.getData()[1]==0x00)&&(message.getData()[0]==0x00) )
        {
            CD_ERROR_NO_HEADER("Error Reset or No Error. canId: %d.\n",canId);
            return true;
        }
        else if ( (message.getData()[1]==0x10)&&(message.getData()[0]==0x00) )
        {
            CD_ERROR_NO_HEADER("Generic error. canId: %d.\n",canId);
            return true;
        }
        else if (message.getData()[1]==0x23)
        {
            if (message.getData()[0]==0x10)
            {
                CD_ERROR_NO_HEADER("Continuous over-current. canId: %d.\n",canId);
                return true;
            }
            else if (message.getData()[0]==0x40)
            {
                CD_ERROR_NO_HEADER("Short-circuit. canId: %d.\n",canId);
                return true;
            }
            CD_ERROR_NO_HEADER("NOT SPECIFIED IN MANUAL. canId: %d.\n",canId);
            return false;
        }
        else if (message.getData()[1]==0x32)
        {
            if (message.getData()[0]==0x10)
            {
                CD_ERROR_NO_HEADER("DC-link over-voltage. canId: %d.\n",canId);
                return true;
            }
            else if (message.getData()[0]==0x20)
            {
                CD_ERROR_NO_HEADER("DC-link under-voltage. canId: %d.\n",canId);
                return true;
            }
            CD_ERROR_NO_HEADER("NOT SPECIFIED IN MANUAL. canId: %d.\n",canId);
            return false;
        }
        else if ( (message.getData()[1]==0x42)&&(message.getData()[0]==0x80) )
        {
            CD_ERROR_NO_HEADER("Over temperature motor. canId: %d.\n",canId);
            return true;
        }
        else if ( (message.getData()[1]==0x43)&&(message.getData()[0]==0x10) )
        {
            CD_ERROR_NO_HEADER("Over temperature drive. canId: %d.\n",canId);
            return true;
        }
        else if (message.getData()[1]==0x54)
        {
            if (message.getData()[0]==0x41)
            {
                CD_ERROR_NO_HEADER("Driver disabled due to enable input. canId: %d.\n",canId);
                return true;
            }
            else if (message.getData()[0]==0x42)
            {
                CD_ERROR_NO_HEADER("Negative limit switch active. canId: %d.\n",canId);
                return true;
            }
            else if (message.getData()[0]==0x43)
            {
                CD_ERROR_NO_HEADER("Positive limit switch active. canId: %d.\n",canId);
                return true;
            }
            CD_ERROR_NO_HEADER("NOT SPECIFIED IN MANUAL. canId: %d.\n",canId);
            return false;
        }
        else if ( (message.getData()[1]==0x61)&&(message.getData()[0]==0x00) )
        {
            CD_ERROR_NO_HEADER("Invalid setup data. canId: %d.\n",canId);
            return true;
        }
        else if ( (message.getData()[1]==0x75)&&(message.getData()[0]==0x00) )
        {
            CD_ERROR_NO_HEADER("Communication error. canId: %d.\n",canId);
            return true;
        }
        else if (message.getData()[1]==0x81)
        {
            if (message.getData()[0]==0x10)
            {
                CD_ERROR_NO_HEADER("CAN overrun (message lost). canId: %d.\n",canId);
                return true;
            }
            else if (message.getData()[0]==0x30)
            {
                CD_ERROR_NO_HEADER("Life guard error or heartbeat error. canId: %d.\n",canId);
                return true;
            }
            CD_ERROR_NO_HEADER("NOT SPECIFIED IN MANUAL. canId: %d.\n",canId);
            return false;
        }
        else if ( (message.getData()[1]==0x83)&&(message.getData()[0]==0x31) )
        {
            CD_ERROR_NO_HEADER("I2t protection triggered. canId: %d.\n",canId);
            return true;
        }
        else if ( (message.getData()[1]==0x85)&&(message.getData()[0]==0x80) )
        {
            CD_ERROR_NO_HEADER("Position wraparound / Hal sensor missing. canId: %d.\n",canId);
            return true;
        }
        else if ( (message.getData()[1]==0x86)&&(message.getData()[0]==0x11) )
        {
            CD_ERROR_NO_HEADER("Control error / Following error. canId: %d.\n",canId);
            return true;
        }
        else if ( (message.getData()[1]==0x90)&&(message.getData()[0]==0x00) )
        {
            CD_ERROR_NO_HEADER("Command error canId: %d.\n",canId);
            return true;
        }
        else if (message.getData()[1]==0xFF)
        {
            if (message.getData()[0]==0x01)
            {
                CD_ERROR_NO_HEADER("Generic interpolated position mode error (PVT / PT error). canId: %d.\n",canId);
                return true;
            }
            else if (message.getData()[0]==0x02)
            {
                CD_ERROR_NO_HEADER("Change set acknowledge bit wrong value. canId: %d.\n",canId);
                return true;
            }
            else if (message.getData()[0]==0x03)
            {
                CD_ERROR_NO_HEADER("Specified homing method not available. canId: %d.\n",canId);
                return true;
            }
            else if (message.getData()[0]==0x04)
            {
                CD_ERROR_NO_HEADER("A wrong mode is set in object 6060h, modes_of_operation. canId: %d.\n",canId);
                return true;
            }
            else if (message.getData()[0]==0x05)
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
