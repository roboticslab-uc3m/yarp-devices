// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TechnosoftIpos.hpp"

#include <bitset>

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

    void interpretSupportedDriveModes(uint32_t data)
    {
        std::bitset<32> bits(data);

        if (bits.test(0))
        {
            CD_INFO("\t*profiled position (pp)\n");
        }
        if (bits.test(1))
        {
            CD_INFO("\t*velocity (vl)\n");
        }
        if (bits.test(2))
        {
            CD_INFO("\t*profiled velocity (pv)\n");
        }
        if (bits.test(3))
        {
            CD_INFO("\t*profiled torque (tq)\n");
        }
        if (bits.test(5))
        {
            CD_INFO("\t*homing (hm)\n");
        }
        if (bits.test(6))
        {
            CD_INFO("\t*interpolated position (ip)\n");
        }
        if (bits.test(7))
        {
            CD_INFO("\t*cyclic synchronous position\n");
        }
        if (bits.test(8))
        {
            CD_INFO("\t*cyclic synchronous velocity\n");
        }
        if (bits.test(9))
        {
            CD_INFO("\t*cyclic synchronous torque\n");
        }
        if (bits.test(16))
        {
            CD_INFO("\t*electronic camming position (manufacturer specific)\n");
        }
        if (bits.test(17))
        {
            CD_INFO("\t*electronic gearing position (manufacturer specific)\n");
        }
        if (bits.test(18))
        {
            CD_INFO("\t*external reference position (manufacturer specific)\n");
        }
        if (bits.test(19))
        {
            CD_INFO("\t*external reference speed (manufacturer specific)\n");
        }
        if (bits.test(20))
        {
            CD_INFO("\t*external reference torque (manufacturer specific)\n");
        }
    }

    inline char getByte(uint32_t number, int n)
    {
        // https://stackoverflow.com/a/7787433
        return (number >> (8 * n)) & 0xFF;
    }
}

// -----------------------------------------------------------------------------

bool roboticslab::TechnosoftIpos::registerSender(CanSenderDelegate * sender)
{
    this->sender = sender;
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
    uint32_t data;

    if (!sdoClient->upload("Device type", &data, 0x1000))
    {
        return false;
    }

    CD_INFO("CiA standard: %d.\n", data & 0xFFFF);

    if (!sdoClient->upload("Supported drive modes", &data, 0x6502))
    {
        return false;
    }

    interpretSupportedDriveModes(data);

    sdoClient->upload("Identity Object: Vendor ID", &data, 0x1018, 0x01);

    if (!sdoClient->upload("Identity Object: Product Code", &data, 0x1018, 0x02))
    {
        return false;
    }

    CD_INFO("Retrieved product code: P%03d.%03d.E%03d.\n", data / 1000000, (data / 1000) % 1000, data % 1000);

    if (!retrieveDrivePeakCurrent(data, &drivePeakCurrent))
    {
        CD_ERROR("Unhandled iPOS model %d, unable to retrieve drive peak current.\n", data);
        return false;
    }

    CD_SUCCESS("Retrieved drive peak current: %f A.\n", drivePeakCurrent);

    if (!sdoClient->upload("Identity Object: Revision number", &data, 0x1018, 0x03))
    {
        return false;
    }

    CD_INFO("Revision number: %c%c%c%c.\n", getByte(data, 3), getByte(data, 2), getByte(data, 1), getByte(data, 0));

    if (!sdoClient->upload("Identity Object: Serial number", &data, 0x1018, 0x04))
    {
        return false;
    }

    CD_INFO("Serial number: %c%c%02x%02x.\n", getByte(data, 3), getByte(data, 2), getByte(data, 1), getByte(data, 0));

    return sdoClient->download<int16_t>("Quick stop option code", 6, 0x605A);
}

// -----------------------------------------------------------------------------
/** -- Start Remote Node: Used to change NMT state of one or all NMT slaves to Operational.
 PDO communication will beallowed. */

bool roboticslab::TechnosoftIpos::start()
{
    // NMT Start Remote Node (to operational, Fig 4.1)
    uint8_t msg_start[] = {0x01, (uint8_t)canId};
    return sender->prepareMessage(message_builder(0, 2, msg_start));
}

// -----------------------------------------------------------------------------

bool roboticslab::TechnosoftIpos::readyToSwitchOn()
{
    uint8_t msg_readyToSwitchOn[] = {0x06,0x00}; //-- readyToSwitchOn, also acts as shutdown.
    // -- send se diferencia de senRaw en que tiene un delay y adems incluye el ID (mirar funcin)
    if( ! this->send( 0x200, 2, msg_readyToSwitchOn) ) // -- 0x200 (valor critico que se pone sin saber que significa) 2 (tamano del mensaje)
    {
        CD_ERROR("Could not send \"readyToSwitchOn/shutdown\". %s\n", CanUtils::msgToStr(canId, 0x200, 2, msg_readyToSwitchOn).c_str() );
        return false;
    }
    CD_SUCCESS("Sent \"readyToSwitchOn/shutdown\". %s\n", CanUtils::msgToStr(canId, 0x200, 2, msg_readyToSwitchOn).c_str() );

    //-- Do not force expect response as only happens upon transition.
    //-- For example, if already on readyToSwitchOn, function would get stuck.

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::TechnosoftIpos::switchOn()
{
    uint8_t msg_switchOn[] = {0x07,0x00};  //-- switchOn, also acts as disableOperation
    if( ! this->send( 0x200, 2, msg_switchOn) )
    {
        CD_ERROR("Could not send \"switchOn/disableOperation\". %s\n", CanUtils::msgToStr(canId, 0x200, 2, msg_switchOn).c_str() );
        return false;
    }
    CD_SUCCESS("Sent \"switchOn/disableOperation\". %s\n", CanUtils::msgToStr(canId, 0x200, 2, msg_switchOn).c_str() );

    //while( (! this->getSwitchOn) ) {
    //    CD_INFO("Waiting for response to \"switchOn/disableOperation\" on id %d...\n", this->canId);
    //    yarp::os::Time::delay(0.1);  //-- [s]
    //}

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::TechnosoftIpos::enable()
{
    uint8_t msg_enable[] = {0x0F,0x00}; // enable

    if( ! this->send( 0x200, 2, msg_enable) )
    {
        CD_ERROR("Could not send \"enable\". %s\n", CanUtils::msgToStr(canId, 0x200, 2, msg_enable).c_str() );
        return false;
    }
    CD_SUCCESS("Sent \"enable\". %s\n", CanUtils::msgToStr(canId, 0x200, 2, msg_enable).c_str() );
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
    uint8_t msg_resetNodes[] = {0x81,0x00};
    return sender->prepareMessage(message_builder(0, 2, msg_resetNodes));
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
        CD_ERROR("Could not send \"reset communication\". %s\n", CanUtils::msgToStr(canId, 0, 2, msg_resetCommunication).c_str() );
        return false;
    }
    CD_SUCCESS("Sent \"reset communication\". %s\n", CanUtils::msgToStr(canId, 0, 2, msg_resetCommunication).c_str() );

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
    // NMT Reset Node (Manual 4.1.2.3)
    uint8_t msg_resetNode[] = {0x81, (uint8_t)id};
    return sender->prepareMessage(message_builder(id, 2, msg_resetNode));
}


// -----------------------------------------------------------------------------

bool roboticslab::TechnosoftIpos::interpretMessage(const yarp::dev::CanMessage & message)
{
    //--------------- Give high priority to PT, override EMCY red -------------------------

    if (message.getData()[0] == 0x01 && message.getData()[1] == 0xFF && message.getData()[2] == 0x01)
    {
        CD_INFO("Interpolated position mode status. canId: %d.\n",canId);

        if ((message.getData()[4] & 0x80) == 0)
        {
            CD_INFO("\t* buffer is not empty.\n");
        }
        else
        {
            CD_INFO("\t* buffer is empty.\n");

            if (linInterpBuffer->getType() == "pvt")
            {
                if ((message.getData()[4] & 0x08) == 0)
                {
                    CD_INFO("\t* pvt maintained position on buffer empty (zero velocity).\n");
                }
                else
                {
                    CD_INFO("\t* pvt performed quick stop on buffer empty (non-zero velocity).\n");
                }
            }
        }

        if ((message.getData()[4] & 0x40) == 0)
        {
            CD_INFO("\t* buffer is not low.\n");
        }
        else
        {
            CD_INFO("\t* buffer is low.\n");
        }

        if ((message.getData()[4] & 0x20) == 0)
        {
            CD_INFO("\t* buffer is not full.\n");
        }
        else
        {
            CD_INFO("\t* buffer is full.\n");
        }

        if ((message.getData()[4] & 0x10) == 0)
        {
            CD_INFO("\t* no integrity counter error.\n");
        }
        else
        {
            CD_INFO("\t* integrity counter error.\n");
        }

        return true;
    }
    else if( (message.getData()[0]==0x37)&&(message.getData()[1]==0x96) )
    {
        CD_WARNING("pt movement ended. canId: %d (via %X).\n",canId,message.getId()-canId);
        return true;
    }
    else if( (message.getId()-canId) == 0x580 )  // -------------- SDO ----------------------
    {
        sdoClient->notify(message.getData(), message.getLen());
        return true;
    }
    else if( (message.getId()-canId) == 0x180 )  // ---------------------- PDO1 ----------------------
    {
        if( (message.getData()[0]==0x37)&&(message.getData()[1]==0x92) )
        {
            CD_INFO("Got PDO1 that it is observed as ack \"start position\" from driver. %s\n",CanUtils::msgToStr(message).c_str());
            return true;
        }
        else if( (message.getData()[0]==0x37)&&(message.getData()[1]==0x86) )
        {
            CD_INFO("Got PDO1 that it is observed when driver arrives to position target. %s\n",CanUtils::msgToStr(message).c_str());
            return true;
        }
        else if( (message.getData()[0]==0x40)&&(message.getData()[1]==0x02) )
        {
            CD_INFO("Got PDO1 that it is observed as TRANSITION performed upon \"start\". %s\n",CanUtils::msgToStr(message).c_str());
            return true;
        }
        else if( (message.getData()[0]==0x40)&&(message.getData()[1]==0x03) )
        {
            CD_INFO("Got PDO1 that it is observed as part of TRANSITION performed upon \"readyToSwitchOn\". %s\n",CanUtils::msgToStr(message).c_str());
            return true;
        }
        else if( (message.getData()[0]==0x21)&&(message.getData()[1]==0x02) )
        {
            CD_INFO("Got PDO1 that it is observed as part of TRANSITION performed upon \"readyToSwitchOn\". %s\n",CanUtils::msgToStr(message).c_str());
            return true;
        }
        else if( (message.getData()[0]==0x21)&&(message.getData()[1]==0x03) )
        {
            CD_INFO("Got PDO1 that it is observed as part of TRANSITION performed upon \"switchOn\". %s\n",CanUtils::msgToStr(message).c_str());
            return true;
        }
        else if( (message.getData()[0]==0x33)&&(message.getData()[1]==0x83) )
        {
            CD_INFO("Got PDO1 that it is observed as part of TRANSITION performed upon \"enable\". %s\n",CanUtils::msgToStr(message).c_str());
            return true;
        }
        CD_INFO("Got PDO1 from driver side: unknown. %s\n",CanUtils::msgToStr(message).c_str());
        return false;
    }
    else if( (message.getId()-canId) == 0x280 )  // PDO2
    {
        if( (message.getData()[0]==0x37)&&(message.getData()[1]==0x92) )
        {
            CD_INFO("Got PDO2 that it is observed as ack \"start position\" from driver. %s\n",CanUtils::msgToStr(message).c_str());
            return true;
        }
        else if( (message.getData()[0]==0x37)&&(message.getData()[1]==0x86) )
        {
            CD_INFO("Got PDO2 that it is observed when driver arrives to position target. %s\n",CanUtils::msgToStr(message).c_str());
            return true;
        }
        else if( (message.getData()[0]==0x40)&&(message.getData()[1]==0x02) )
        {
            CD_INFO("Got PDO2 that it is observed as TRANSITION performed upon \"start\". %s\n",CanUtils::msgToStr(message).c_str());
            return true;
        }
        else if( (message.getData()[0]==0x40)&&(message.getData()[1]==0x03) )
        {
            CD_INFO("Got PDO2 that it is observed as part of TRANSITION performed upon \"readyToSwitchOn\". %s\n",CanUtils::msgToStr(message).c_str());
            return true;
        }
        else if( (message.getData()[0]==0x21)&&(message.getData()[1]==0x02) )
        {
            CD_INFO("Got PDO2 that it is observed as part of TRANSITION performed upon \"readyToSwitchOn\". %s\n",CanUtils::msgToStr(message).c_str());
            return true;
        }
        else if( (message.getData()[0]==0x21)&&(message.getData()[1]==0x03) )
        {
            CD_INFO("Got PDO2 that it is observed as part of TRANSITION performed upon \"switchOn\". %s\n",CanUtils::msgToStr(message).c_str());
            return true;
        }
        else if( (message.getData()[0]==0x83)&&(message.getData()[1]==0x83) )
        {
            CD_INFO("Got PDO2 that it is observed as part of TRANSITION performed upon \"enable\". %s\n",CanUtils::msgToStr(message).c_str());
            return true;
        }
        CD_INFO("Got PDO2 from driver side: unknown. %s\n",CanUtils::msgToStr(message).c_str());
        return false;
    }
    else if( (message.getId()-canId) == 0x80 )  // EMERGENCY (EMCY), Table 4.2 Emergency Error Codes (p57, 73/263)
    {
        CD_ERROR("Got EMERGENCY from iPOS. %s ",CanUtils::msgToStr(message).c_str());
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

    CD_ERROR("Unknown message: %s\n", CanUtils::msgToStr(message).c_str());

    return false;

}  //-- ends interpretMessage

// -----------------------------------------------------------------------------
