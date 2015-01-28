// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "MotorIpos.hpp"

// ------------------ IPositionDirect Related ----------------------------------

bool teo::MotorIpos::setPositionDirectModeRaw() {
    CD_INFO("\n");

    //-- ptprepare: pg. 165 (181/263)
    //*************************************************************
    //-- 1. - 4. From start to enable.
    //*************************************************************
    //-- 5. Disable the RPDO3. Write zero in object 1602 h sub-index 0, this will disable the PDO.
    //-- Send the following message (SDO access to object 1602 h sub-index 0, 8-bit value 0):
    uint8_t disableRPDO3[]={0x2F,0x02,0x16,0x00,0x00,0x00,0x00,0x00};
    if ( ! send(0x600,8,disableRPDO3) )
        return false;
    //*************************************************************
    //-- 6. Map the new objects.
    //-- a) Write in object 1602 h sub-index 1 the description of the interpolated data record
    //-- sub-index 1:
    //-- Send the following message (SDO access to object 1602 h sub-index 1, 32-bit value 60C10120 h ):
    uint8_t mapSDOsub1[]={0x23,0x02,0x16,0x01,0x20,0x01,0xC1,0x60};
    if ( ! send(0x600,8,mapSDOsub1) )
        return false;
    //* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
    //-- b) Write in object 1601 h sub-index 2 the description of the interpolated data record
    //-- sub-index 2:
    //-- Send the following message (SDO access to object 1602 h sub-index 2, 32-bit value 60C10220 h ):
    uint8_t mapSDOsub2[]={0x23,0x02,0x16,0x02,0x20,0x02,0xC1,0x60};
    if ( ! send(0x600,8,mapSDOsub2) )
        return false;
    //*************************************************************
    //-- 7. Enable the RPDO3. Set the object 1601 h sub-index 0 with the value 2.
    //-- Send the following message (SDO access to object 1601 h sub-index 0, 8-bit value 2):
    uint8_t enableRPDO3[]={0x2F,0x02,0x16,0x00,0x02,0x00,0x00,0x00};
    if ( !  send(0x600,8,enableRPDO3) )
        return false;
    //*************************************************************
    //-- 8. Mode of operation. Select interpolation position mode.
    //-- Send the following message (SDO access to object 6060 h , 8-bit value 7 h ):
    uint8_t opMode[]={0x2F,0x60,0x60,0x00,0x07,0x00,0x00,0x00};
    if ( ! send(0x600,8,opMode) )
        return false;
    //*************************************************************
    //-- 9. Interpolation sub mode select. Select PT interpolation position mode.
    //-- Send the following message (SDO access to object 60C0 h , 16-bit value 0000 h ):
    uint8_t subMode[]={0x2E,0xC0,0x60,0x00,0x00,0x00,0x00,0x00};
    if ( ! send(0x600,8,subMode) )
        return false;
    //*************************************************************
    //-- 10. Interpolated position buffer length. Set the buffer length to 12. The maximum length is 15.
    //uint8_t buffLength[]={0x2B,0x74,0x20,0x00,0x00,0x0C,0x00,0x00};  //-- 12
    uint8_t buffLength[]={0x2B,0x74,0x20,0x00,0x00,0x0F,0x00,0x00};  //-- 15
    if ( ! send(0x600,8,buffLength) )
        return false;
    //*************************************************************
    //-- 11. Interpolated position buffer configuration. By setting the value A001 h , the buffer is
    //-- cleared and the integrity counter will be set to 1. Send the following message (SDO
    //-- access to object 2074 h , 16-bit value C h ):
    uint8_t buffConf[]={0x2B,0x74,0x20,0x00,0x01,0xA0,0x00,0x00};
    if ( ! send(0x600,8,buffConf) )
        return false;
    //*************************************************************
    //-- 12. Interpolated position initial position. Set the initial position to 0.5 rotations. By using a
    //-- 500 lines incremental encoder the corresponding value of object 2079 h expressed in
    //-- encoder counts is (1000 d ) 3E8 h . By using the settings done so far, if the final position
    //-- command were to be 0, the drive would travel to (Actual position – 1000).
    //-- Send the following message (SDO access to object 2079 h , 32-bit value 0 h ):
    //uint8_t initPos[]={0x23,0x79,0x20,0x00,0xE8,0x03,0x00,0x00};  // Put 3E8 h.
    uint8_t initPos[]={0x23,0x79,0x20,0x00,0x00,0x00,0x00,0x00};  // Put 0 h instead.
    if ( ! send(0x600,8,initPos) )
        return false;
    //*************************************************************

    return true;
}

// -----------------------------------------------------------------------------

bool teo::MotorIpos::setPositionRaw(int j, double ref) {

    //CD_INFO("j:%d ref:%f\n",j,refs[j]);
    //*************************************************************
    //-- 13. Send the 1 st PT point.
    //-- Position= 20000 IU (0x00004E20)
    //-- Time = 1000 IU (0x03E8)
    //-- IC = 1 (0x01)
    //-- 1IU = 1 encoder pulse
    //-- 1IU = 1 control loop = 1ms by default
    //-- IC=Integrity Counter
    //-- The drive motor will do 10 rotations (20000 counts) in 1000 milliseconds.
    //-- Send the following message:
    //uint8_t ptpoint1[]={0x20,0x4E,0x00,0x00,0xE8,0x03,0x00,0x02};
    uint8_t msg_ptPoint[8];
    int32_t position = ref * this->tr * 11.11112;  // Appply tr & convert units to encoder increments
    memcpy(msg_ptPoint+0,&position,4);
    memcpy(msg_ptPoint+4,&(this->ptModeMs),2);
    uint8_t ic = (ptPointCounter+1)*2;  //-- *2 because only 7 bits, so only half the buffer is usable
    memcpy(msg_ptPoint+7,&ic,1);

    yarp::os::Time::delay(0.001); // 0.01 okay for 1 arm (6 motors), had to reduce to 0.0075 for two arms (12 motors)
    ptBuffer.wait();
    if ( ! send(0x400,8,msg_ptPoint) ) {
        CD_ERROR("msg_ptPoint in %d",canId);
        return false;
    }
    CD_SUCCESS("Sent to canId %d: pos %f, time %d, ic %d.\n",canId,ref,ptModeMs,ic);
    ptBuffer.post();

    ptPointCounter++;


    //*************************************************************
    uint8_t startPT[]={0x1F,0x00};
    //Send start

    if( ptPointCounter == 7 ) {  //-- Put max buffer length here (usually 7).

        if( ! send(0x200,2,startPT) )
        {
            CD_ERROR("Could not send \"startPT\". %s\n", msgToStr(0x200,2,startPT).c_str() );
            return false;
        }
        CD_SUCCESS("Sent \"startPT\". %s\n", msgToStr(0x200,2,startPT).c_str() );
    }

    return true;
}

// -----------------------------------------------------------------------------

bool teo::MotorIpos::setPositionsRaw(const int n_joint, const int *joints, double *refs) {
    //CD_INFO("n_joint:%d, drivers.size():" CD_SIZE_T "\n",n_joint,drivers.size());

    //CD_INFO("j:%d ref:%f\n",j,refs[j]);
    //*************************************************************
    //-- 13. Send the 1 st PT point.
    //-- Position= 20000 IU (0x00004E20)
    //-- Time = 1000 IU (0x03E8)
    //-- IC = 1 (0x01)
    //-- 1IU = 1 encoder pulse
    //-- 1IU = 1 control loop = 1ms by default
    //-- IC=Integrity Counter
    //-- The drive motor will do 10 rotations (20000 counts) in 1000 milliseconds.
    //-- Send the following message:
    //uint8_t ptpoint1[]={0x20,0x4E,0x00,0x00,0xE8,0x03,0x00,0x02};
    uint8_t msg_ptPoint[8];
    int32_t position = refs[0] * this->tr * 11.11112;  // Appply tr & convert units to encoder increments
    memcpy(msg_ptPoint+0,&position,4);
    memcpy(msg_ptPoint+4,&(this->ptModeMs),2);
    uint8_t ic = (ptPointCounter+1)*2;  //-- *2 because only 7 bits, so only half the buffer is usable
    memcpy(msg_ptPoint+7,&ic,1);

    yarp::os::Time::delay(0.001); // 0.01 okay for 1 arm (6 motors), had to reduce to 0.0075 for two arms (12 motors)
    ptBuffer.wait();
    if ( ! send(0x400,8,msg_ptPoint) ) {
        CD_ERROR("msg_ptPoint in %d",canId);
        return false;
    }
    CD_SUCCESS("Sent to canId %d: pos %f, time %d, ic %d.\n",canId,refs[0],ptModeMs,ic);
    ptBuffer.post();

    ptPointCounter++;


    //*************************************************************
    uint8_t startPT[]={0x1F,0x00};
    //Send start

    if( ptPointCounter == 7 ) {  //-- Put max buffer length here (usually 7).

        if( ! send(0x200,2,startPT) )
        {
            CD_ERROR("Could not send \"startPT\". %s\n", msgToStr(0x200,2,startPT).c_str() );
            return false;
        }
        CD_SUCCESS("Sent \"startPT\". %s\n", msgToStr(0x200,2,startPT).c_str() );
    }

    //*************************************************************
    /*uint8_t stopPT[]={0x0F,0x00};

    for(int j=0;j<drivers.size();j++)
    {
        CD_INFO("Wait for canId %d...\n",drivers[j]->getCanId());

        while( ! drivers[j]->ptMovementDone );

        yarp::os::Time::delay(0.01);
        if( ! drivers[j]->send(0x200,2,stopPT) )
        {
            CD_ERROR("Could not send \"stopPT\" to canId: %d.\n",drivers[j]->getCanId());
            return false;
        }
        CD_SUCCESS("Sent \"stopPT\" to canId: %d.\n",drivers[j]->getCanId());

    }*/
    //*************************************************************

    return true;
}

// -----------------------------------------------------------------------------

bool teo::MotorIpos::setPositionsRaw(const double *refs) {
    return true;
}

// -----------------------------------------------------------------------------
