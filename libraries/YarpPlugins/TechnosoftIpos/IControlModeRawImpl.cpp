// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TechnosoftIpos.hpp"

#include <cstring>

#include <yarp/os/Vocab.h>

// ############################## IControlModeRaw Related ##############################

bool roboticslab::TechnosoftIpos::setPositionModeRaw(int j)
{
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( j != 0 ) return false;

    //*************************************************************
    uint8_t msg_position_mode[] = {0x2F,0x60,0x60,0x00,0x01,0x00,0x00,0x00};  // Position mode

    if( ! send( 0x600, 5, msg_position_mode) )
    {
        CD_ERROR("Could not send \"position_mode\". %s\n", msgToStr(0x600, 5, msg_position_mode).c_str() );
        return false;
    }
    CD_SUCCESS("Sent \"position_mode\". %s\n", msgToStr(0x600, 5, msg_position_mode).c_str() );

    if (!sdoSemaphore->await(msg_position_mode))
    {
        CD_ERROR("Did not receive \"position_mode\" ack. %s\n", msgToStr(0x600, 5, msg_position_mode).c_str());
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::TechnosoftIpos::setVelocityModeRaw(int j)
{
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( j != 0 ) return false;

    //*************************************************************
    uint8_t msg_velocity_mode[]= {0x2F,0x60,0x60,0x00,0x03,0x00,0x00,0x00}; // Velocity mode

    if( ! send( 0x600, 5, msg_velocity_mode) )
    {
        CD_ERROR("Could not send \"vel_mode\". %s\n", msgToStr(0x600, 5, msg_velocity_mode).c_str() );
        return false;
    }
    CD_SUCCESS("Sent \"vel_mode\". %s\n", msgToStr(0x600, 5, msg_velocity_mode).c_str() );

    if (!sdoSemaphore->await(msg_velocity_mode))
    {
        CD_ERROR("Did not receive \"vel_mode\" ack. %s\n", msgToStr(0x600, 5, msg_velocity_mode).c_str());
        return false;
    }

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

    if (!sdoSemaphore->await(msg_ref_type))
    {
        CD_ERROR("Did not receive \"ref_type\" ack. %s\n", msgToStr(0x600, 8, msg_ref_type).c_str());
        return false;
    }

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

    if (!sdoSemaphore->await(msg_mode_torque))
    {
        CD_ERROR("Did not receive \"mode_torque\" ack. %s\n", msgToStr(0x600, 8, msg_mode_torque).c_str());
        return false;
    }

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

    linInterpBuffer->resetIntegrityCounter();

    //-- ptprepare: pg. 168 (184/263)
    //*************************************************************
    //-- 1. - 4. From start to enable.
    //*************************************************************
    //-- 5. Disable the RPDO3. Write zero in object 1602 h sub-index 0, this will disable the PDO.
    //-- Send the following message (SDO access to object 1602 h sub-index 0, 8-bit value 0):
    uint8_t disableRPDO3[]= {0x2F,0x02,0x16,0x00,0x00,0x00,0x00,0x00};
    if ( ! send(0x600,8,disableRPDO3) || !sdoSemaphore->await(disableRPDO3) )
        return false;
    //*************************************************************
    //-- 6. Map the new objects.
    //-- a) Write in object 1602 h sub-index 1 the description of the interpolated data record
    //-- sub-index 1:
    //-- Send the following message (SDO access to object 1602 h sub-index 1, 32-bit value 60C10120 h ):
    uint8_t mapSDOsub1[]= {0x23,0x02,0x16,0x01,0x20,0x01,0xC1,0x60};
    if ( ! send(0x600,8,mapSDOsub1) || !sdoSemaphore->await(mapSDOsub1) )
        return false;
    //* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
    //-- b) Write in object 1602 h sub-index 2 the description of the interpolated data record
    //-- sub-index 2:
    //-- Send the following message (SDO access to object 1602 h sub-index 2, 32-bit value 60C10220 h ):
    uint8_t mapSDOsub2[]= {0x23,0x02,0x16,0x02,0x20,0x02,0xC1,0x60};
    if ( ! send(0x600,8,mapSDOsub2) || !sdoSemaphore->await(mapSDOsub2) )
        return false;
    //*************************************************************
    //-- 7. Enable the RPDO3. Set the object 1602 h sub-index 0 with the value 2.
    //-- Send the following message (SDO access to object 1601 h sub-index 0, 8-bit value 2):
    uint8_t enableRPDO3[]= {0x2F,0x02,0x16,0x00,0x02,0x00,0x00,0x00};
    if ( !  send(0x600,8,enableRPDO3) || !sdoSemaphore->await(enableRPDO3) )
        return false;
    //*************************************************************
    //-- 8. Mode of operation. Select interpolation position mode.
    //-- Send the following message (SDO access to object 6060 h , 8-bit value 7 h ):
    uint8_t opMode[]= {0x2F,0x60,0x60,0x00,0x07,0x00,0x00,0x00};
    if ( ! send(0x600,8,opMode) || !sdoSemaphore->await(opMode) )
        return false;
    //*************************************************************
    //-- 9. Interpolation sub mode select. Select PVT interpolation position mode.
    //-- Send the following message (SDO access to object 60C0 h , 16-bit value FFFF h ):
    uint8_t subMode[]= {0x2E,0xC0,0x60,0x00,0x00,0x00,0x00,0x00};
    linInterpBuffer->configureSubMode(subMode);
    if ( ! send(0x600,8,subMode) || !sdoSemaphore->await(subMode) )
        return false;
    //*************************************************************
    //-- 10. Interpolated position buffer length. (...)
    uint8_t buffLength[]= {0x2B,0x73,0x20,0x00,0x00,0x00,0x00,0x00};
    linInterpBuffer->configureBufferSize(buffLength);
    if ( ! send(0x600,8,buffLength) || !sdoSemaphore->await(buffLength) )
        return false;
    //*************************************************************
    //-- 11. Interpolated position buffer configuration. By setting the value A001 h , the buffer is
    //-- cleared and the integrity counter will be set to 1. Send the following message (SDO
    //-- access to object 2074 h , 16-bit value C h ):
    uint8_t buffConf[]= {0x2B,0x74,0x20,0x00,0x00,0xA0,0x00,0x00};
    if ( ! send(0x600,8,buffConf) || !sdoSemaphore->await(buffConf) )
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
    if (!getEncoderRaw(0, &ref)) return false;
    int position = ref * this->tr * (encoderPulses / 360.0);  // Appply tr & convert units to encoder increments
    memcpy(initPos+4,&position,4); //Copy block of memory
    if ( ! send(0x600,8,initPos) || !sdoSemaphore->await(initPos) )
        return false;

    yarp::os::Time::delay(0.1);  //-- Seems like a "must".

    linInterpBuffer->setInitialReference(ref);
    linInterpBuffer->updateTarget(ref);

    for (int i = 0; i < linInterpBuffer->getBufferSize(); i++)
    {
        if (!sendLinearInterpolationTarget())
        {
            CD_ERROR("Unable to send point %d/%d to buffer.\n", i + 1, linInterpBuffer->getBufferSize());
            return false;
        }
    }

    return true;
}

/*************************************************************************/
// -----------------------------------------------------------------------------

bool roboticslab::TechnosoftIpos::getControlModeRaw(int j, int *mode)
{
    //CD_INFO("(%d)\n",j);  //-- Too verbose in controlboardwrapper2 stream

    //-- Check index within range
    if ( j != 0 ) return false;

    bool ok = true;
    ok &= getControlModeRaw1(mode);
    ok &= getControlModeRaw2();
    ok &= getControlModeRaw3();
    ok &= getControlModeRaw4();

    return ok;
}

/******************* getControlModeRaw Splited **********************/
bool roboticslab::TechnosoftIpos::getControlModeRaw1(int *mode)
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

    if (!sdoSemaphore->await(msgOperationDisplay))
    {
        CD_ERROR("Did not receive modes of operation display response. %s\n", msgToStr(0x600, 8, msgOperationDisplay).c_str());
        return false;
    }

    int8_t got;
    std::memcpy(&got, msgOperationDisplay + 4, 1);

    int temp = VOCAB_CM_UNKNOWN;

    switch (got)
    {
    // handled
    case -5:
        CD_INFO("\t-iPOS specific: External Reference Torque Mode. canId: %d.\n", canId);
        temp = modeCurrentTorque == VOCAB_CM_TORQUE ? VOCAB_CM_TORQUE : VOCAB_CM_CURRENT;
        break;
    case 1:
        CD_INFO("\t-Profile Position Mode. canId: %d.\n", canId);
        temp = VOCAB_CM_POSITION;
        break;
    case 3:
        CD_INFO("\t-Profile Velocity Mode. canId: %d.\n", canId);
        temp = VOCAB_CM_VELOCITY;
        break;
    case 7:
        CD_INFO("\t-Interpolated Position Mode. canId: %d.\n", canId);
        temp = VOCAB_CM_POSITION_DIRECT;
        break;
    // unhandled
    case -4:
        CD_INFO("\t-iPOS specific: External Reference Speed Mode. canId: %d.\n", canId);
        break;
    case -3:
        CD_INFO("\t-iPOS specific: External Reference Position Mode. canId: %d.\n", canId);
        break;
    case -2:
        CD_INFO("\t-iPOS specific: Electronic Camming Position Mode. canId: %d.\n", canId);
        break;
    case -1:
        CD_INFO("\t-iPOS specific: Electronic Gearing Position Mode. canId: %d.\n", canId);
        break;
    case 6:
        CD_INFO("\t-Homing Mode. canId: %d.\n", canId);
        break;
    case 8:
        CD_INFO("\t-Cyclic Synchronous Position Mode. canId: %d.\n", canId);
        break;
    default:
        CD_WARNING("\t-Mode \"%d\" not specified in manual, may be in Fault or not enabled yet. canId(%d).\n", got, canId);
        break;
    }

    *mode = temp;

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

    if (!sdoSemaphore->await(msgManuStatus))
    {
        CD_ERROR("Did not receive manufacturer status response. %s\n", msgToStr(0x600, 8, msgManuStatus).c_str());
        return false;
    }

    if(msgManuStatus[4] & 1) //0000 0001 (bit 0)
    {
        CD_INFO("\t-Ready to switch on. canId: %d.\n", canId);
    }
    if(msgManuStatus[4] & 2) //0000 0010 (bit 1)
    {
        CD_INFO("\t-Switched on. canId: %d.\n", canId);
    }
    if(msgManuStatus[4] & 4) //0000 0100 (bit 2)
    {
        CD_INFO("\t-Operation Enabled. canId: %d.\n", canId);
    }
    if(msgManuStatus[4] & 8) //0000 1000 (bit 3)
    {
        CD_INFO("\t-Fault. If set, a fault condition is or was present in the drive. canId: %d.\n", canId);
    }
    if(msgManuStatus[4] & 16) //0001 0000 (bit 4)
    {
        CD_INFO("\t-Motor supply voltage is present. canId: %d.\n", canId);//true
    }
    else
    {
        CD_INFO("\t-Motor supply voltage is absent. canId: %d.\n", canId);//false
    }
    if(!(msgManuStatus[4] & 32)) //0010 0000 (bit 5), negated.
    {
        CD_INFO("\t-Performing a quick stop. canId: %d.\n", canId);
    }
    if(msgManuStatus[4] & 64) //0100 0000 (bit 6)
    {
        CD_INFO("\t-Switch on disabled. canId: %d.\n", canId);
    }
    if(msgManuStatus[4] & 128) //1000 0000 (bit 7)
    {
        CD_INFO("\t-Warning. A TML function / homing was called, while another TML function / homing is still in execution. The last call is ignored. canId: %d.\n", canId);
    }
    if(msgManuStatus[5] & 1) //(bit 8)
    {
        CD_INFO("\t-A TML function or homing is executed. Until the function or homing execution ends or is aborted, no other TML function / homing may be called. canId: %d.\n", canId);
    }
    if(msgManuStatus[5] & 2) //(bit 9)
    {
        CD_INFO("\t-Remote: drive parameters may be modified via CAN and the drive will execute the command message. canId: %d.\n", canId); // true
    }
    else
    {
        CD_INFO("\t-Remote: drive is in local mode and will not execute the command message (only TML internal)."); // false
    }
    if(msgManuStatus[5] & 4) //(bit 10)
    {
        CD_INFO("\t-Target reached. canId: %d.\n", canId);  // true
    }
    else
    {
        CD_INFO("\t-Target not reached. canId: %d.\n", canId);  // false (improvised, not in manual, but reasonable).
    }
    if(msgManuStatus[5] & 8) //(bit 11)
    {
        CD_INFO("\t-Internal Limit Active. canId: %d.\n", canId);
    }
    if(msgManuStatus[5] & 64) //(bit 14)
    {
        CD_INFO("\t-Last event set has ocurred. canId: %d.\n", canId); // true
    }
    else
    {
        CD_INFO("\t-No event set or the programmed event has not occurred yet. canId: %d.\n", canId); // false
    }
    if(msgManuStatus[5] & 128) //(bit 15)
    {
        CD_INFO("\t-Axis on. Power stage is enabled. Motor control is performed. canId: %d.\n", canId); // true
    }
    else
    {
        CD_INFO("\t-Axis off. Power stage is disabled. Motor control is not performed. canId: %d.\n", canId); // false
    }
    ////Much much more in Table 5.6
    if(msgManuStatus[6] & 1) //(bit 16)
    {
        CD_INFO("\t*Drive/motor initialization performed. canId: %d.\n", canId);
    }
    if(msgManuStatus[6] & 2) //(bit 17)
    {
        CD_INFO("\t*Position trigger 1 reached. canId: %d.\n", canId);
    }
    if(msgManuStatus[6] & 4) //(bit 18)
    {
        CD_INFO("\t*Position trigger 2 reached. canId: %d.\n", canId);
    }
    if(msgManuStatus[6] & 8) //(bit 19)
    {
        CD_INFO("\t*Position trigger 3 reached. canId: %d.\n", canId);
    }
    if(msgManuStatus[6] & 16) //(bit 20)
    {
        CD_INFO("\t*Position trigger 4 reached. canId: %d.\n", canId);
    }
    if(msgManuStatus[6] & 32) //(bit 21)
    {
        CD_INFO("\t*AUTORUN mode enabled. canId: %d.\n", canId);
    }
    if(msgManuStatus[6] & 64) //(bit 22)
    {
        CD_INFO("\t*Limit switch positive event / interrupt triggered. canId: %d.\n", canId);
    }
    if(msgManuStatus[6] & 128) //(bit 23)
    {
        CD_INFO("\t*Limit switch negative event / interrupt triggered. canId: %d.\n", canId);
    }
    if(msgManuStatus[7] & 1) //(bit 24)
    {
        CD_INFO("\t*Capture event/interrupt triggered. canId: %d.\n", canId);
    }
    if(msgManuStatus[7] & 2) //(bit 25)
    {
        CD_INFO("\t*Target command reached. canId: %d.\n", canId);
    }
    if(msgManuStatus[7] & 4) //(bit 26)
    {
        CD_INFO("\t*Motor I2t protection warning level reached. canId: %d.\n", canId);
    }
    if(msgManuStatus[7] & 8) //(bit 27)
    {
        CD_INFO("\t*Drive I2t protection warning level reached. canId: %d.\n", canId);
    }
    if(msgManuStatus[7] & 16) //(bit 28)
    {
        CD_INFO("\t*Gear ratio in electronic gearing mode reached. canId: %d.\n", canId);
    }
    if(msgManuStatus[7] & 64) //(bit 30)
    {
        CD_INFO("\t*Reference position in absolute electronic camming mode reached. canId: %d.\n", canId);
    }
    if(msgManuStatus[7] & 128) //(bit 31)
    {
        CD_INFO("\t*Drive/motor in fault status. canId: %d.\n", canId);
    }

    return true;
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

    if (!sdoSemaphore->await(msgError))
    {
        CD_ERROR("Did not receive Motion Error Register response. %s\n", msgToStr(0x600, 8, msgError).c_str());
        return false;
    }

    if (msgError[4] & 1) //0000 0001 (bit 0)
    {
        CD_INFO("\t*CAN error. Set when CAN controller is in error mode. canId: %d.\n", canId);
    }
    if (msgError[4] & 2) //0000 0010 (bit 1)
    {
        CD_INFO("\t*Short-circuit. Set when protection is triggered. canId: %d.\n", canId);
    }
    if (msgError[4] & 4) //0000 0100 (bit 2)
    {
        CD_INFO("\t*Invalid setup data. Set when the EEPROM stored setup data is not valid or not present. canId: %d.\n", canId);
    }
    if (msgError[4] & 8) //0000 1000 (bit 3)
    {
        CD_INFO("\t*Control error (position/speed error too big). Set when protection is triggered. canId: %d.\n", canId);
    }
    if (msgError[4] & 16) //0001 0000 (bit 4)
    {
        CD_INFO("\t*Communication error. Set when protection is triggered. canId: %d.\n", canId);//true
    }
    if (msgError[4] & 32) //0010 0000 (bit 5)
    {
        CD_INFO("\t*Motor position wraps around. Set when protection is triggered. canId: %d.\n", canId);
    }
    if (msgError[4] & 64) //0100 0000 (bit 6)
    {
        CD_INFO("\t*Positive limit switch active. Set when LSP input is in active state. canId: %d.\n", canId);
    }
    if (msgError[4] & 128) //1000 0000 (bit 7)
    {
        CD_INFO("\t*Negative limit switch active. Set when LSN input is in active state. canId: %d.\n", canId);
    }
    if (msgError[5] & 1) //(bit 8)
    {
        CD_INFO("\t*Over current. Set when protection is triggered. canId: %d.\n", canId);
    }
    if (msgError[5] & 2) //(bit 9)
    {
        CD_INFO("\t*I2t protection. Set when protection is triggered. canId: %d.\n", canId);
    }
    if (msgError[5] & 4) //(bit 10)
    {
        CD_INFO("\t*Over temperature motor. Set when protection is triggered. canId: %d.\n", canId);
    }
    if (msgError[5] & 8) //(bit 11)
    {
        CD_INFO("\t*Over temperature drive. Set when protection is triggered. canId: %d.\n", canId);
    }
    if (msgError[5] & 16) //(bit 12)
    {
        CD_INFO("\t*Over-voltage. Set when protection is triggered. canId: %d.\n", canId);
    }
    if (msgError[5] & 32) //(bit 13)
    {
        CD_INFO("\t*Under-voltage. Set when protection is triggered. canId: %d.\n", canId);
    }
    if (msgError[5] & 64) //(bit 14)
    {
        CD_INFO("\t*Command error. This bit is set in several situations. They can be distinguished either by the associated emergency code, or in conjunction with other bits. canId: %d.\n", canId);
    }
    if (msgError[5] & 128) //(bit 15)
    {
        CD_INFO("\t*Drive disabled due to enable input. Set when enable input is on disable state. canId: %d.\n", canId);
    }

    return true;
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

    if (!sdoSemaphore->await(msgErrorDetail))
    {
        CD_ERROR("Did not receive Detailed Error Register response. %s\n", msgToStr(0x600, 8, msgErrorDetail).c_str());
        return false;
    }

    if(msgErrorDetail[4] & 1) //0000 0001 (bit 0)
    {
        CD_INFO("\t**The number of nested function calls exceeded the length of TML stack. Last function call was ignored. canId: %d.\n", canId);
    }
    if(msgErrorDetail[4] & 2) //0000 0010 (bit 1)
    {
        CD_INFO("\t**A RET/RETI instruction was executed while no function/ISR was active. canId: %d.\n", canId);
    }
    if(msgErrorDetail[4] & 4) //0000 0100 (bit 2)
    {
        CD_INFO("\t**A call to an inexistent homing routine was received. canId: %d.\n", canId);
    }
    if(msgErrorDetail[4] & 8) //0000 1000 (bit 3)
    {
        CD_INFO("\t**A call to an inexistent function was received. canId: %d.\n", canId);
    }
    if(msgErrorDetail[4] & 16) //0001 0000 (bit 4)
    {
        CD_INFO("\t**UPD instruction received while AXISON was executed. The UPD instruction was ingnored and it must be sent again when AXISON is completed. canId: %d.\n", canId);
    }
    if(msgErrorDetail[4] & 32) //0010 0000 (bit 5)
    {
        CD_INFO("\t**Cancelable call instruction received while another cancelable function was active. canId: %d.\n", canId);
    }
    if(msgErrorDetail[4] & 64) //0100 0000 (bit 6)
    {
        CD_INFO("\t**Positive software limit switch is active. canId: %d.\n", canId);
    }
    if(msgErrorDetail[4] & 128) //1000 0000 (bit 7)
    {
        CD_INFO("\t**Negative software limit switch is active. canId: %d.\n", canId);
    }
    if(msgErrorDetail[5] & 1) //(bit 8)
    {
        CD_INFO("\t**S-curve parameters caused and invalid profile. UPD instruction was ignored. canId: %d.\n", canId);
    }
    if(msgErrorDetail[5] & 2) //(bit 9)
    {
        CD_INFO("\t**Update ignored for S-curve. canId: %d.\n", canId);
    }
    if(msgErrorDetail[5] & 4) //(bit 10)
    {
        CD_INFO("\t**Encoder broken wire. canId: %d.\n", canId);
    }
    if(msgErrorDetail[5] & 8) //(bit 11)
    {
        CD_INFO("\t**Motionless start failed. canId: %d.\n", canId);
    }
    if(msgErrorDetail[5] & 32) //(bit 13)
    {
        CD_INFO("\t**Self check error. canId: %d.\n", canId);
    }

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
    CD_DEBUG("(%d, %s)\n", j, yarp::os::Vocab::decode(mode).c_str());

    //-- Check index within range
    if (j != 0) return false;

    switch (mode)
    {
    case VOCAB_CM_POSITION:
        return setPositionModeRaw(j);
    case VOCAB_CM_VELOCITY:
        return setVelocityModeRaw(j);
    case VOCAB_CM_CURRENT:
    case VOCAB_CM_TORQUE:
        modeCurrentTorque = mode;
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
