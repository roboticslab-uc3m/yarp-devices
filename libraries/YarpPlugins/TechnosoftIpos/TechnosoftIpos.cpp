// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TechnosoftIpos.hpp"

#include <cstring>

// -----------------------------------------------------------------------------

std::string roboticslab::TechnosoftIpos::msgToStr(yarp::dev::CanMessage * message)
{
    std::stringstream tmp;
    for(int i=0; i < message->getLen()-1; i++)
    {
        tmp << std::hex << static_cast<int>(message->getData()[i]) << " ";
    }
    tmp << std::hex << static_cast<int>(message->getData()[message->getLen()-1]);
    tmp << ". canId(";
    tmp << std::dec << (message->getId() & 0x7F);
    tmp << ") via(";
    tmp << std::hex << (message->getId() & 0xFF80);
    tmp << ").";
    return tmp.str();
}

// -----------------------------------------------------------------------------

std::string roboticslab::TechnosoftIpos::msgToStr(uint32_t cob, uint16_t len, uint8_t * msgData)
{
    std::stringstream tmp;
    for(int i=0; i < len-1; i++)
    {
        tmp << std::hex << static_cast<int>(*(msgData+i)) << " ";
    }
    tmp << std::hex << static_cast<int>(*(msgData+len-1));
    tmp << ". canId(";
    tmp << std::dec << canId;
    tmp << ") via(";
    tmp << std::hex << cob;
    tmp << ").";
    return tmp.str();
}

// -----------------------------------------------------------------------------

bool roboticslab::TechnosoftIpos::send(uint32_t cob, uint16_t len, uint8_t * msgData)
{

    if ( (lastUsage - yarp::os::Time::now()) < DELAY )
        yarp::os::Time::delay( lastUsage + DELAY - yarp::os::Time::now() );

    yarp::dev::CanMessage &msg = canOutputBuffer[0];
    msg.setId(cob + canId);
    msg.setLen(len);
    std::memcpy(msg.getData(), msgData, len * sizeof(uint8_t));

    unsigned int sent;

    if( ! canDevicePtr->canWrite(canOutputBuffer, 1, &sent, true) )
        return false;

    lastUsage = yarp::os::Time::now();
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::TechnosoftIpos::setPositionDirectModeRaw()
{
    CD_INFO("\n");

    //-- ptprepare: pg. 165 (181/263)
    //*************************************************************
    //-- 1. - 4. From start to enable.
    //*************************************************************
    //-- 5. Disable the RPDO3. Write zero in object 1602 h sub-index 0, this will disable the PDO.
    //-- Send the following message (SDO access to object 1602 h sub-index 0, 8-bit value 0):
    uint8_t disableRPDO3[]= {0x2F,0x02,0x16,0x00,0x00,0x00,0x00,0x00};
    if ( ! send(0x600,8,disableRPDO3) )
        return false;
    //*************************************************************
    //-- 6. Map the new objects.
    //-- a) Write in object 1602 h sub-index 1 the description of the interpolated data record
    //-- sub-index 1:
    //-- Send the following message (SDO access to object 1602 h sub-index 1, 32-bit value 60C10120 h ):
    uint8_t mapSDOsub1[]= {0x23,0x02,0x16,0x01,0x20,0x01,0xC1,0x60};
    if ( ! send(0x600,8,mapSDOsub1) )
        return false;
    //* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
    //-- b) Write in object 1601 h sub-index 2 the description of the interpolated data record
    //-- sub-index 2:
    //-- Send the following message (SDO access to object 1602 h sub-index 2, 32-bit value 60C10220 h ):
    uint8_t mapSDOsub2[]= {0x23,0x02,0x16,0x02,0x20,0x02,0xC1,0x60};
    if ( ! send(0x600,8,mapSDOsub2) )
        return false;
    //*************************************************************
    //-- 7. Enable the RPDO3. Set the object 1601 h sub-index 0 with the value 2.
    //-- Send the following message (SDO access to object 1601 h sub-index 0, 8-bit value 2):
    uint8_t enableRPDO3[]= {0x2F,0x02,0x16,0x00,0x02,0x00,0x00,0x00};
    if ( !  send(0x600,8,enableRPDO3) )
        return false;
    //*************************************************************
    //-- 8. Mode of operation. Select interpolation position mode.
    //-- Send the following message (SDO access to object 6060 h , 8-bit value 7 h ):
    uint8_t opMode[]= {0x2F,0x60,0x60,0x00,0x07,0x00,0x00,0x00};
    if ( ! send(0x600,8,opMode) )
        return false;
    //*************************************************************
    //-- 9. Interpolation sub mode select. Select PT interpolation position mode.
    //-- Send the following message (SDO access to object 60C0 h , 16-bit value 0000 h ):
    uint8_t subMode[]= {0x2E,0xC0,0x60,0x00,0x00,0x00,0x00,0x00};
    if ( ! send(0x600,8,subMode) )
        return false;
    //*************************************************************
    //-- 10. Interpolated position buffer length. Set the buffer length to 12. The maximum length is 15.
    //uint8_t buffLength[]={0x2B,0x74,0x20,0x00,0x00,0x0C,0x00,0x00};  //-- 12
    uint8_t buffLength[]= {0x2B,0x74,0x20,0x00,0x00,0x0F,0x00,0x00}; //-- 15
    if ( ! send(0x600,8,buffLength) )
        return false;
    //*************************************************************
    //-- 11. Interpolated position buffer configuration. By setting the value A001 h , the buffer is
    //-- cleared and the integrity counter will be set to 1. Send the following message (SDO
    //-- access to object 2074 h , 16-bit value C h ):
    uint8_t buffConf[]= {0x2B,0x74,0x20,0x00,0x01,0xA0,0x00,0x00};
    if ( ! send(0x600,8,buffConf) )
        return false;
    //*************************************************************
    //-- 12. Interpolated position initial position. Set the initial position to 0.5 rotations. By using a
    //-- 500 lines incremental encoder the corresponding value of object 2079 h expressed in
    //-- encoder counts is (1000 d ) 3E8 h . By using the settings done so far, if the final position
    //-- command were to be 0, the drive would travel to (Actual position – 1000).
    //-- Send the following message (SDO access to object 2079 h , 32-bit value 0 h ):
    //uint8_t initPos[]={0x23,0x79,0x20,0x00,0xE8,0x03,0x00,0x00};  // Put 3E8 h.
    uint8_t initPos[]= {0x23,0x79,0x20,0x00,0x00,0x00,0x00,0x00}; // Put 0 h instead.
    if ( ! send(0x600,8,initPos) )
        return false;
    //*************************************************************

    yarp::os::Time::delay(1);  //-- Seems like a "must".

    return true;
}

// -----------------------------------------------------------------------------
