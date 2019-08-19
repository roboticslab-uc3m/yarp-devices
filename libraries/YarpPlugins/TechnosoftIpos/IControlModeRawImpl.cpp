// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TechnosoftIpos.hpp"

#include <cstring>

#include <bitset>

#include <yarp/os/Vocab.h>

// ############################## IControlModeRaw Related ##############################

bool roboticslab::TechnosoftIpos::setPositionModeRaw(int j)
{
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( j != 0 ) return false;

    return sdoClient->download<uint8_t>("Modes of Operation", 1, 0x6060);
}

// -----------------------------------------------------------------------------

bool roboticslab::TechnosoftIpos::setVelocityModeRaw(int j)
{
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( j != 0 ) return false;

    return sdoClient->download<uint8_t>("Modes of Operation", 3, 0x6060);
}

// -----------------------------------------------------------------------------

bool roboticslab::TechnosoftIpos::setTorqueModeRaw(int j)
{
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( j != 0 ) return false;

    bool ok = true;
    ok = ok && sdoClient->download<uint16_t>("External Reference Type", 1, 0x201D);
    ok = ok && sdoClient->download<uint8_t>("Modes of Operation", -5, 0x6060);

    if (!ok)
    {
        return false;
    }

    //-- Control word (manual 215 of 263).
    uint8_t msg_torque_word[] = {0x1F,0x00};

    if( ! send( 0x200, 2, msg_torque_word) )
    {
        CD_ERROR("Could not send msg_torque_word. %s\n", CanUtils::msgToStr(canId, 0x200, 2, msg_torque_word).c_str() );
        return false;
    }
    CD_SUCCESS("Sent \"torque_word\". %s\n", CanUtils::msgToStr(canId, 0x200, 2, msg_torque_word).c_str() );

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
    if (!sdoClient->download<uint8_t>("RPDO3 Mapping Parameter", 0, 0x1602))
    {
        return false;
    }
    //*************************************************************
    //-- 6. Map the new objects.
    //-- a) Write in object 1602 h sub-index 1 the description of the interpolated data record
    //-- sub-index 1:
    //-- Send the following message (SDO access to object 1602 h sub-index 1, 32-bit value 60C10120 h ):
    if (!sdoClient->download<uint32_t>("RPDO3 Mapping Parameter: 1st mapped object", 0x60C10120, 0x1602, 0x01))
    {
        return false;
    }
    //* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
    //-- b) Write in object 1602 h sub-index 2 the description of the interpolated data record
    //-- sub-index 2:
    //-- Send the following message (SDO access to object 1602 h sub-index 2, 32-bit value 60C10220 h ):
    if (!sdoClient->download<uint32_t>("RPDO3 Mapping Parameter: 2nd mapped object", 0x60C10220, 0x1602, 0x02))
    {
        return false;
    }
    //*************************************************************
    //-- 7. Enable the RPDO3. Set the object 1602 h sub-index 0 with the value 2.
    //-- Send the following message (SDO access to object 1601 h sub-index 0, 8-bit value 2):
    if (!sdoClient->download<uint8_t>("RPDO3 Mapping Parameter", 2, 0x1602))
    {
        return false;
    }
    //*************************************************************
    //-- 8. Mode of operation. Select interpolation position mode.
    //-- Send the following message (SDO access to object 6060 h , 8-bit value 7 h ):
    if (!sdoClient->download<uint8_t>("Modes of Operation", 7, 0x6060))
    {
        return false;
    }
    //*************************************************************
    //-- 9. Interpolation sub mode select. Select PVT interpolation position mode.
    //-- Send the following message (SDO access to object 60C0 h , 16-bit value FFFF h ):
    if (!sdoClient->download<int16_t>("Interpolation sub mode select", linInterpBuffer->getSubMode(), 0x60C0))
    {
        return false;
    }
    //*************************************************************
    //-- 10. Interpolated position buffer length. (...)
    if (!sdoClient->download<uint16_t>("Interpolated position buffer length", linInterpBuffer->getBufferSize(), 0x2073))
    {
        return false;
    }
    //*************************************************************
    //-- 11. Interpolated position buffer configuration. By setting the value A001 h , the buffer is
    //-- cleared and the integrity counter will be set to 1. Send the following message (SDO
    //-- access to object 2074 h , 16-bit value C h ):
    if (!sdoClient->download<uint16_t>("Interpolated position buffer configuration", 0xA000, 0x2074))
    {
        return false;
    }
    //*************************************************************
    //-- 12. Interpolated position initial position. Set the initial position to 0.5 rotations. By using a
    //-- 500 lines incremental encoder the corresponding value of object 2079 h expressed in
    //-- encoder counts is (1000 d ) 3E8 h . By using the settings done so far, if the final position
    //-- command were to be 0, the drive would travel to (Actual position – 1000).
    //-- Send the following message (SDO access to object 2079 h , 32-bit value 0 h ):

    double ref;
    if (!getEncoderRaw(0, &ref)) return false;
    int32_t data = ref * tr * (encoderPulses / 360.0);

    if (!sdoClient->download("Interpolated position initial position", data, 0x2079))
    {
        return false;
    }

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
    int8_t data;

    if (!sdoClient->upload("Modes of Operation Display", &data, 0x6061))
    {
        return false;
    }

    int temp = VOCAB_CM_UNKNOWN;

    switch (data)
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
        CD_WARNING("\t-Mode \"%d\" not specified in manual, may be in Fault or not enabled yet. canId(%d).\n", data, canId);
        break;
    }

    *mode = temp;

    return true;
}

//*************************************************************
bool roboticslab::TechnosoftIpos::getControlModeRaw2()
{
    uint32_t data;

    if (!sdoClient->upload("Manufacturer status register", &data, 0x1002))
    {
        return false;
    }

    std::bitset<32> bits(data);

    if (bits.test(0))
    {
        CD_INFO("\t-Ready to switch on. canId: %d.\n", canId);
    }
    if (bits.test(1))
    {
        CD_INFO("\t-Switched on. canId: %d.\n", canId);
    }
    if (bits.test(2))
    {
        CD_INFO("\t-Operation Enabled. canId: %d.\n", canId);
    }
    if (bits.test(3))
    {
        CD_INFO("\t-Fault. If set, a fault condition is or was present in the drive. canId: %d.\n", canId);
    }
    if (bits.test(4))
    {
        CD_INFO("\t-Motor supply voltage is present. canId: %d.\n", canId);
    }
    else
    {
        CD_INFO("\t-Motor supply voltage is absent. canId: %d.\n", canId);
    }
    if (!bits.test(5)) // negated
    {
        CD_INFO("\t-Performing a quick stop. canId: %d.\n", canId);
    }
    if (bits.test(6))
    {
        CD_INFO("\t-Switch on disabled. canId: %d.\n", canId);
    }
    if (bits.test(7))
    {
        CD_INFO("\t-Warning. A TML function / homing was called, while another TML function / homing is still in execution. The last call is ignored. canId: %d.\n", canId);
    }
    if (bits.test(8))
    {
        CD_INFO("\t-A TML function or homing is executed. Until the function or homing execution ends or is aborted, no other TML function / homing may be called. canId: %d.\n", canId);
    }
    if (bits.test(9))
    {
        CD_INFO("\t-Remote: drive parameters may be modified via CAN and the drive will execute the command message. canId: %d.\n", canId);
    }
    else
    {
        CD_INFO("\t-Remote: drive is in local mode and will not execute the command message (only TML internal).");
    }
    if (bits.test(10))
    {
        CD_INFO("\t-Target reached. canId: %d.\n", canId);
    }
    else
    {
        CD_INFO("\t-Target not reached. canId: %d.\n", canId); // improvised, not in manual, but reasonable
    }
    if (bits.test(11))
    {
        CD_INFO("\t-Internal Limit Active. canId: %d.\n", canId);
    }
    if (bits.test(14))
    {
        CD_INFO("\t-Last event set has ocurred. canId: %d.\n", canId);
    }
    else
    {
        CD_INFO("\t-No event set or the programmed event has not occurred yet. canId: %d.\n", canId);
    }
    if (bits.test(15))
    {
        CD_INFO("\t-Axis on. Power stage is enabled. Motor control is performed. canId: %d.\n", canId);
    }
    else
    {
        CD_INFO("\t-Axis off. Power stage is disabled. Motor control is not performed. canId: %d.\n", canId);
    }
    ////Much much more in Table 5.6
    if (bits.test(16))
    {
        CD_INFO("\t*Drive/motor initialization performed. canId: %d.\n", canId);
    }
    if (bits.test(17))
    {
        CD_INFO("\t*Position trigger 1 reached. canId: %d.\n", canId);
    }
    if (bits.test(18))
    {
        CD_INFO("\t*Position trigger 2 reached. canId: %d.\n", canId);
    }
    if (bits.test(19))
    {
        CD_INFO("\t*Position trigger 3 reached. canId: %d.\n", canId);
    }
    if (bits.test(20))
    {
        CD_INFO("\t*Position trigger 4 reached. canId: %d.\n", canId);
    }
    if (bits.test(21))
    {
        CD_INFO("\t*AUTORUN mode enabled. canId: %d.\n", canId);
    }
    if (bits.test(22))
    {
        CD_INFO("\t*Limit switch positive event / interrupt triggered. canId: %d.\n", canId);
    }
    if (bits.test(23))
    {
        CD_INFO("\t*Limit switch negative event / interrupt triggered. canId: %d.\n", canId);
    }
    if (bits.test(24))
    {
        CD_INFO("\t*Capture event/interrupt triggered. canId: %d.\n", canId);
    }
    if (bits.test(25))
    {
        CD_INFO("\t*Target command reached. canId: %d.\n", canId);
    }
    if (bits.test(26))
    {
        CD_INFO("\t*Motor I2t protection warning level reached. canId: %d.\n", canId);
    }
    if (bits.test(27))
    {
        CD_INFO("\t*Drive I2t protection warning level reached. canId: %d.\n", canId);
    }
    if (bits.test(28))
    {
        CD_INFO("\t*Gear ratio in electronic gearing mode reached. canId: %d.\n", canId);
    }
    if (bits.test(30))
    {
        CD_INFO("\t*Reference position in absolute electronic camming mode reached. canId: %d.\n", canId);
    }
    if (bits.test(31))
    {
        CD_INFO("\t*Drive/motor in fault status. canId: %d.\n", canId);
    }

    return true;
}

bool roboticslab::TechnosoftIpos::getControlModeRaw3()
{
    uint16_t data;

    if (!sdoClient->upload("Motion Error Register", &data, 0x2000))
    {
        return false;
    }

    std::bitset<16> bits(data);

    if (bits.test(0))
    {
        CD_INFO("\t*CAN error. Set when CAN controller is in error mode. canId: %d.\n", canId);
    }
    if (bits.test(1))
    {
        CD_INFO("\t*Short-circuit. Set when protection is triggered. canId: %d.\n", canId);
    }
    if (bits.test(2))
    {
        CD_INFO("\t*Invalid setup data. Set when the EEPROM stored setup data is not valid or not present. canId: %d.\n", canId);
    }
    if (bits.test(3))
    {
        CD_INFO("\t*Control error (position/speed error too big). Set when protection is triggered. canId: %d.\n", canId);
    }
    if (bits.test(4))
    {
        CD_INFO("\t*Communication error. Set when protection is triggered. canId: %d.\n", canId);//true
    }
    if (bits.test(5))
    {
        CD_INFO("\t*Motor position wraps around. Set when protection is triggered. canId: %d.\n", canId);
    }
    if (bits.test(6))
    {
        CD_INFO("\t*Positive limit switch active. Set when LSP input is in active state. canId: %d.\n", canId);
    }
    if (bits.test(7))
    {
        CD_INFO("\t*Negative limit switch active. Set when LSN input is in active state. canId: %d.\n", canId);
    }
    if (bits.test(8))
    {
        CD_INFO("\t*Over current. Set when protection is triggered. canId: %d.\n", canId);
    }
    if (bits.test(9))
    {
        CD_INFO("\t*I2t protection. Set when protection is triggered. canId: %d.\n", canId);
    }
    if (bits.test(10))
    {
        CD_INFO("\t*Over temperature motor. Set when protection is triggered. canId: %d.\n", canId);
    }
    if (bits.test(11))
    {
        CD_INFO("\t*Over temperature drive. Set when protection is triggered. canId: %d.\n", canId);
    }
    if (bits.test(12))
    {
        CD_INFO("\t*Over-voltage. Set when protection is triggered. canId: %d.\n", canId);
    }
    if (bits.test(13))
    {
        CD_INFO("\t*Under-voltage. Set when protection is triggered. canId: %d.\n", canId);
    }
    if (bits.test(14))
    {
        CD_INFO("\t*Command error. This bit is set in several situations. They can be distinguished either by the associated emergency code, or in conjunction with other bits. canId: %d.\n", canId);
    }
    if (bits.test(15))
    {
        CD_INFO("\t*Drive disabled due to enable input. Set when enable input is on disable state. canId: %d.\n", canId);
    }

    return true;
}

bool roboticslab::TechnosoftIpos::getControlModeRaw4()
{
    uint16_t data;

    if (!sdoClient->upload("Detailed Error Register", &data, 0x2002))
    {
        return false;
    }

    std::bitset<16> bits(data);

    if (bits.test(0))
    {
        CD_INFO("\t**The number of nested function calls exceeded the length of TML stack. Last function call was ignored. canId: %d.\n", canId);
    }
    if (bits.test(1))
    {
        CD_INFO("\t**A RET/RETI instruction was executed while no function/ISR was active. canId: %d.\n", canId);
    }
    if (bits.test(2))
    {
        CD_INFO("\t**A call to an inexistent homing routine was received. canId: %d.\n", canId);
    }
    if (bits.test(3))
    {
        CD_INFO("\t**A call to an inexistent function was received. canId: %d.\n", canId);
    }
    if (bits.test(4))
    {
        CD_INFO("\t**UPD instruction received while AXISON was executed. The UPD instruction was ingnored and it must be sent again when AXISON is completed. canId: %d.\n", canId);
    }
    if (bits.test(5))
    {
        CD_INFO("\t**Cancelable call instruction received while another cancelable function was active. canId: %d.\n", canId);
    }
    if (bits.test(6))
    {
        CD_INFO("\t**Positive software limit switch is active. canId: %d.\n", canId);
    }
    if (bits.test(7))
    {
        CD_INFO("\t**Negative software limit switch is active. canId: %d.\n", canId);
    }
    if (bits.test(8))
    {
        CD_INFO("\t**S-curve parameters caused and invalid profile. UPD instruction was ignored. canId: %d.\n", canId);
    }
    if (bits.test(9))
    {
        CD_INFO("\t**Update ignored for S-curve. canId: %d.\n", canId);
    }
    if (bits.test(10))
    {
        CD_INFO("\t**Encoder broken wire. canId: %d.\n", canId);
    }
    if (bits.test(11))
    {
        CD_INFO("\t**Motionless start failed. canId: %d.\n", canId);
    }
    if (bits.test(13))
    {
        CD_INFO("\t**Self check error. canId: %d.\n", canId);
    }

    return true;
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
