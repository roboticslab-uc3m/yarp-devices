// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TechnosoftIpos.hpp"

// ############################## IControlModeRaw Related ##############################

bool roboticslab::TechnosoftIpos::setPositionModeRaw(int j)
{
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( j != 0 ) return false;

    //*************************************************************
    uint8_t msg_position_mode[] = {0x2F,0x60,0x60,0x00,0x01};  // Position mode

    if( ! send( 0x600, 5, msg_position_mode) )
    {
        CD_ERROR("Could not send \"position_mode\". %s\n", msgToStr(0x600, 5, msg_position_mode).c_str() );
        return false;
    }
    CD_SUCCESS("Sent \"position_mode\". %s\n", msgToStr(0x600, 5, msg_position_mode).c_str() );
    //*************************************************************

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::TechnosoftIpos::setVelocityModeRaw(int j)
{
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( j != 0 ) return false;

    //*************************************************************
    uint8_t msg_velocity_mode[]= {0x2F,0x60,0x60,0x00,0x03}; // Velocity mode

    if( ! send( 0x600, 5, msg_velocity_mode) )
    {
        CD_ERROR("Could not send \"vel_mode\". %s\n", msgToStr(0x600, 5, msg_velocity_mode).c_str() );
        return false;
    }
    CD_SUCCESS("Sent \"vel_mode\". %s\n", msgToStr(0x600, 5, msg_velocity_mode).c_str() );
    //*************************************************************

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::TechnosoftIpos::setTorqueModeRaw(int j)
{
    CD_INFO("(%d)\n",j);
    bool ok = true;
    //-- Check index within range
    if ( j != 0 ) return false;

    // -- splited functions
    ok &= setTorqueModeRaw1();
    ok &= setTorqueModeRaw2();
    ok &= setTorqueModeRaw3();

    return ok;
}

/******************* setTorqueModeRaw Splited **********************/

bool roboticslab::TechnosoftIpos::setTorqueModeRaw1()
{

    //-- 5. External reference type. Slave receives reference through CAN (manual 208 of 263).
    uint8_t msg_ref_type[]= {0x2B,0x1D,0x20,0x00,0x01,0x00,0x00,0x00}; //CAN

    if( ! send( 0x600, 8, msg_ref_type) )
    {
        CD_ERROR("Could not send \"ref_type\". %s\n", msgToStr(0x600, 8, msg_ref_type).c_str() );
        return false;
    }
    CD_SUCCESS("Sent \"ref_type\". %s\n", msgToStr(0x600, 8, msg_ref_type).c_str() );

    return true;
}

bool roboticslab::TechnosoftIpos::setTorqueModeRaw2()
{

    //-- Mode -5 (manual 209 of 263). Send the following message (SDO access to object 6060 h , 8-bit value -1)
    uint8_t msg_mode_torque[]= {0x2F,0x60,0x60,0x00,0xFB,0x00,0x00,0x00};

    if( ! send( 0x600, 8, msg_mode_torque) )
    {
        CD_ERROR("Could not send \"mode_torque\". %s\n", msgToStr(0x600, 8, msg_mode_torque).c_str() );
        return false;
    }
    CD_SUCCESS("Sent \"mode_torque\". %s\n", msgToStr(0x600, 8, msg_mode_torque).c_str() );

    return true;
}

bool roboticslab::TechnosoftIpos::setTorqueModeRaw3()
{

    //-- Control word (manual 215 of 263).
    uint8_t msg_torque_word[] = {0x1F,0x00};

    if( ! send( 0x200, 2, msg_torque_word) )
    {
        CD_ERROR("Could not send msg_torque_word. %s\n", msgToStr(0x200, 2, msg_torque_word).c_str() );
        return false;
    }
    CD_SUCCESS("Sent \"torque_word\". %s\n", msgToStr(0x200, 2, msg_torque_word).c_str() );

    return true;
}

/*************************************************************************/
// -----------------------------------------------------------------------------

bool roboticslab::TechnosoftIpos::setPositionDirectModeRaw()
{
    CD_INFO("\n");

    pvtPointCounter = 0;

    //-- ptprepare: pg. 168 (184/263)
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
    //-- b) Write in object 1602 h sub-index 2 the description of the interpolated data record
    //-- sub-index 2:
    //-- Send the following message (SDO access to object 1602 h sub-index 2, 32-bit value 60C10220 h ):
    uint8_t mapSDOsub2[]= {0x23,0x02,0x16,0x02,0x20,0x02,0xC1,0x60};
    if ( ! send(0x600,8,mapSDOsub2) )
        return false;
    //*************************************************************
    //-- 7. Enable the RPDO3. Set the object 1602 h sub-index 0 with the value 2.
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
    //-- 9. Interpolation sub mode select. Select PVT interpolation position mode.
    //-- Send the following message (SDO access to object 60C0 h , 16-bit value FFFF h ):
    uint8_t subMode[]= {0x2E,0xC0,0x60,0x00,0xFF,0xFF,0x00,0x00};
    if ( ! send(0x600,8,subMode) )
        return false;
    //*************************************************************
    //-- 10. Interpolated position buffer length. (...)
    uint8_t buffLength[]= {0x2B,0x73,0x20,0x00,0x01,0x00,0x00,0x00};
    if ( ! send(0x600,8,buffLength) )
        return false;
    //*************************************************************
    //-- 11. Interpolated position buffer configuration. By setting the value A001 h , the buffer is
    //-- cleared and the integrity counter will be set to 1. Send the following message (SDO
    //-- access to object 2074 h , 16-bit value C h ):
    uint8_t buffConf[]= {0x2B,0x74,0x20,0x00,0x01,0xC0,0x00,0x00};
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

    double ref;
    getEncoderRaw(0, &ref);
    int position = ref * this->tr * (encoderPulses / 360.0);  // Appply tr & convert units to encoder increments
    memcpy(initPos+4,&position,4); //Copy block of memory
    if ( ! send(0x600,8,initPos) )
        return false;

    yarp::os::Time::delay(1);  //-- Seems like a "must".

    pvtThread->setInitialPose(ref);
    pvtThread->updateTarget(ref);
    pvtThread->step();
    pvtThread->step();

    if (!pvtThread->start())
    {
        CD_ERROR("Unable to start PVT thread.\n");
        return false;
    }

    yarp::os::Time::delay(PVT_MODE_MS * 0.001 / 2);

    uint8_t startPT[]= {0x1F,0x00};

    if( ! send(0x200,2,startPT) )
    {
        CD_ERROR("Could not send \"startPT\". %s\n", msgToStr(0x200,2,startPT).c_str() );
        return false;
    }
    CD_SUCCESS("Sent \"startPT\". %s\n", msgToStr(0x200,2,startPT).c_str() );

    return true;
}

/*************************************************************************/
// -----------------------------------------------------------------------------

bool roboticslab::TechnosoftIpos::getControlModeRaw(int j, int *mode)
{
    //CD_INFO("(%d)\n",j);  //-- Too verbose in controlboardwrapper2 stream
    bool ok = true;
    //-- Check index within range
    if ( j != 0 ) return false;
    ok &= getControlModeRaw1();
    ok &= getControlModeRaw2();
    ok &= getControlModeRaw3();
    ok &= getControlModeRaw4();
    getModeReady.wait();
    *mode = getMode; // -- activate the sending of message mode
    getModeReady.post();

    return ok;
}

/******************* getControlModeRaw Splited **********************/
bool roboticslab::TechnosoftIpos::getControlModeRaw1()
{
    //*************************************************************
    uint8_t msgOperationDisplay[] = {0x40,0x61,0x60,0x00,0x00,0x00,0x00,0x00}; // Manual 6061h: Modes of Operation display
    if( ! send( 0x600, 8, msgOperationDisplay))
    {
        CD_ERROR("Could not send modes of operation display. %s\n", msgToStr(0x600, 8, msgOperationDisplay).c_str() );
        return false;
    }
    CD_SUCCESS("Sent modes of operation display. %s\n", msgToStr(0x600, 8, msgOperationDisplay).c_str() );
    //*************************************************************

    //* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
    yarp::os::Time::delay(DELAY);  // Must delay as it will be from same driver.
    //* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *

    return true;
}

//*************************************************************
bool roboticslab::TechnosoftIpos::getControlModeRaw2()
{
    //-- Ya de paso...
    //*************************************************************
    //uint8_t msgStatusDisplay[] = {0x40,0x41,0x60,0x00,0x00,0x00,0x00,0x00}; // Manual 6041h: Status display word
    //if( ! send( 0x600, 8, msgStatus))
    //{
    //    CD_ERROR("Could not send status display query. %s\n", msgToStr(0x600, 8, msgStatusDisplay).c_str() );
    //    return false;
    //}
    //CD_SUCCESS("Sent status display query. %s\n", msgToStr(0x600, 8, msgStatusDisplay).c_str() );
    //*************************************************************

    //-- Ya de paso, aun mejor...
    //*************************************************************
    uint8_t msgManuStatus[] = {0x40,0x02,0x10,0x00,0x00,0x00,0x00,0x00}; // Manual 1002h contains "6041h Status word" plus Table 5.6
    if( ! send( 0x600, 8, msgManuStatus))
    {
        CD_ERROR("Could not send manufacturer status query. %s\n", msgToStr(0x600, 8, msgManuStatus).c_str() );
        return false;
    }
    CD_SUCCESS("Sent manufacturer status query. %s\n", msgToStr(0x600, 8, msgManuStatus).c_str() );

    return true;
    /***************************************************************/
}

bool roboticslab::TechnosoftIpos::getControlModeRaw3()
{
    //-- Y ya de paso, por qué no...
    //*************************************************************
    uint8_t msgError[] = {0x40,0x00,0x20,0x00,0x00,0x00,0x00,0x00}; // Manual 2000h: Motion Error Register
    if( ! send( 0x600, 8, msgError))
    {
        CD_ERROR("Could not send Motion Error Register query. %s\n", msgToStr(0x600, 8, msgError).c_str() );
        return false;
    }
    CD_SUCCESS("Sent Motion Error Register query. %s\n", msgToStr(0x600, 8, msgError).c_str() );

    return true;
    //*************************************************************
}

bool roboticslab::TechnosoftIpos::getControlModeRaw4()
{
    //-- Y tb ya de paso, por qué no... // no info
    //*************************************************************
    uint8_t msgErrorDetail[] = {0x40,0x02,0x20,0x00,0x00,0x00,0x00,0x00}; // Manual 2002h: Detailed Error Register
    if( ! send( 0x600, 8, msgErrorDetail))
    {
        CD_ERROR("Could not send Detailed Error Register query. %s\n", msgToStr(0x600, 8, msgErrorDetail).c_str() );
        return false;
    }
    CD_SUCCESS("Sent Detailed Error Register query. %s\n", msgToStr(0x600, 8, msgErrorDetail).c_str() );

    return true;
    //*************************************************************

    //-- Y... // no info (just a 000Fh)
    //*************************************************************
    /*uint8_t msgSw[] = {0x40,0x0A,0x10,0x00,0x00,0x00,0x00,0x00}; // Manual 100Ah: Manufacturer Software Version
    if( ! send( 0x600, 8, msgSw))
    {
        CD_ERROR("Could not send Manufacturer Software Version query. %s\n", msgToStr(0x600, 8, msgSw).c_str() );
        return false;
    }
    CD_SUCCESS("Sent Manufacturer Software Version query. %s\n", msgToStr(0x600, 8, msgSw).c_str() );*/
    //*************************************************************
}

/********************************************************************/
// -----------------------------------------------------------------------------

bool roboticslab::TechnosoftIpos::getControlModesRaw(int *modes)
{
    CD_ERROR("Missing implementation\n");
    return false;
}

// ############################## IControlMode2Raw Related ##############################

bool roboticslab::TechnosoftIpos::getControlModesRaw(const int n_joint, const int *joints, int *modes)
{
    CD_DEBUG("\n");

    //-- Check array size
    if ( n_joint != 1 ) return false;

    return getControlModeRaw(0, &modes[0]);
}

// -----------------------------------------------------------------------------

bool roboticslab::TechnosoftIpos::setControlModeRaw(const int j, const int mode)
{
    CD_DEBUG("(%d, %d)\n", j, mode);

    //-- Check index within range
    if (j != 0) return false;

    if (pvtThread->isRunning())
    {
        pvtThread->stop();
    }

    switch (mode)
    {
    case VOCAB_CM_POSITION:
        return setPositionModeRaw(j);
    case VOCAB_CM_VELOCITY:
        return setVelocityModeRaw(j);
    case VOCAB_CM_CURRENT:
    case VOCAB_CM_TORQUE:
        return setTorqueModeRaw(j);
    case VOCAB_CM_POSITION_DIRECT:
        return setPositionDirectModeRaw();
    default:
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::TechnosoftIpos::setControlModesRaw(const int n_joint, const int *joints, int *modes)
{
    CD_DEBUG("(%d)\n",n_joint);

    //-- Check array size
    if ( n_joint != 1 ) return false;

    return setControlModeRaw(0, modes[0]);
}

// -----------------------------------------------------------------------------

bool roboticslab::TechnosoftIpos::setControlModesRaw(int *modes)
{
    CD_DEBUG("\n");
    return setControlModeRaw(0, modes[0]);
}
