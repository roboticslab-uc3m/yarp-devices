// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TechnosoftIpos.hpp"

#include <bitset>

#include <yarp/os/Vocab.h>

#include <ColorDebug.h>

#include "CanUtils.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool TechnosoftIpos::setPositionDirectModeRaw()
{
    CD_DEBUG("\n");

    linInterpBuffer->resetIntegrityCounter();

    PdoConfiguration rpdo3Conf;
    rpdo3Conf.addMapping<uint32_t>(0x60C1, 0x01);
    rpdo3Conf.addMapping<uint32_t>(0x60C1, 0x02);

    bool ok = true;
    ok = ok && can->rpdo3()->configure(rpdo3Conf);
    ok = ok && can->sdo()->download<int8_t>("Modes of Operation", 7, 0x6060);
    ok = ok && can->sdo()->download<int16_t>("Interpolation sub mode select", linInterpBuffer->getSubMode(), 0x60C0);
    ok = ok && can->sdo()->download<uint16_t>("Interpolated position buffer length", linInterpBuffer->getBufferSize(), 0x2073);
    ok = ok && can->sdo()->download<uint16_t>("Interpolated position buffer configuration", 0xA000, 0x2074);
    if (!ok) return false;

    double ref;
    if (!getEncoderRaw(0, &ref)) return false;
    int32_t data = vars.degreesToInternalUnits(ref);

    if (!can->sdo()->download("Interpolated position initial position", data, 0x2079))
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

// -----------------------------------------------------------------------------

bool TechnosoftIpos::getControlModeRaw(int j, int * mode)
{
    //CD_DEBUG("(%d)\n",j); //-- Too verbose in controlboardwrapper2 stream
    CHECK_JOINT(j);

    bool ok = true;
    ok &= getControlModeRaw1(mode);
    ok &= getControlModeRaw2();
    ok &= getControlModeRaw3();
    ok &= getControlModeRaw4();

    return ok;
}

// -----------------------------------------------------------------------------

bool TechnosoftIpos::getControlModeRaw1(int * mode)
{
    int8_t data;

    if (!can->sdo()->upload("Modes of Operation Display", &data, 0x6061))
    {
        return false;
    }

    int temp = VOCAB_CM_UNKNOWN;

    switch (data)
    {
    // handled
    case -5:
        CD_INFO("\t-iPOS specific: External Reference Torque Mode. canId: %d.\n", can->getId());
        temp = vars.controlMode == VOCAB_CM_TORQUE ? VOCAB_CM_TORQUE : VOCAB_CM_CURRENT;
        break;
    case 1:
        CD_INFO("\t-Profile Position Mode. canId: %d.\n", can->getId());
        temp = VOCAB_CM_POSITION;
        break;
    case 3:
        CD_INFO("\t-Profile Velocity Mode. canId: %d.\n", can->getId());
        temp = VOCAB_CM_VELOCITY;
        break;
    case 7:
        CD_INFO("\t-Interpolated Position Mode. canId: %d.\n", can->getId());
        temp = VOCAB_CM_POSITION_DIRECT;
        break;
    // unhandled
    case -4:
        CD_INFO("\t-iPOS specific: External Reference Speed Mode. canId: %d.\n", can->getId());
        break;
    case -3:
        CD_INFO("\t-iPOS specific: External Reference Position Mode. canId: %d.\n", can->getId());
        break;
    case -2:
        CD_INFO("\t-iPOS specific: Electronic Camming Position Mode. canId: %d.\n", can->getId());
        break;
    case -1:
        CD_INFO("\t-iPOS specific: Electronic Gearing Position Mode. canId: %d.\n", can->getId());
        break;
    case 6:
        CD_INFO("\t-Homing Mode. canId: %d.\n", can->getId());
        break;
    case 8:
        CD_INFO("\t-Cyclic Synchronous Position Mode. canId: %d.\n", can->getId());
        break;
    default:
        CD_WARNING("\t-Mode \"%d\" not specified in manual, may be in Fault or not enabled yet. canId(%d).\n", data, can->getId());
        break;
    }

    *mode = temp;
    return true;
}

// -----------------------------------------------------------------------------

bool TechnosoftIpos::getControlModeRaw2()
{
    uint32_t data;

    if (!can->sdo()->upload("Manufacturer status register", &data, 0x1002))
    {
        return false;
    }

    std::bitset<32> bits(data);

    if (bits.test(0))
    {
        CD_INFO("\t-Ready to switch on. canId: %d.\n", can->getId());
    }
    if (bits.test(1))
    {
        CD_INFO("\t-Switched on. canId: %d.\n", can->getId());
    }
    if (bits.test(2))
    {
        CD_INFO("\t-Operation Enabled. canId: %d.\n", can->getId());
    }
    if (bits.test(3))
    {
        CD_INFO("\t-Fault. If set, a fault condition is or was present in the drive. canId: %d.\n", can->getId());
    }
    if (bits.test(4))
    {
        CD_INFO("\t-Motor supply voltage is present. canId: %d.\n", can->getId());
    }
    else
    {
        CD_INFO("\t-Motor supply voltage is absent. canId: %d.\n", can->getId());
    }
    if (!bits.test(5)) // negated
    {
        CD_INFO("\t-Performing a quick stop. canId: %d.\n", can->getId());
    }
    if (bits.test(6))
    {
        CD_INFO("\t-Switch on disabled. canId: %d.\n", can->getId());
    }
    if (bits.test(7))
    {
        CD_INFO("\t-Warning. A TML function / homing was called, while another TML function / homing is still in execution. The last call is ignored. canId: %d.\n", can->getId());
    }
    if (bits.test(8))
    {
        CD_INFO("\t-A TML function or homing is executed. Until the function or homing execution ends or is aborted, no other TML function / homing may be called. canId: %d.\n", can->getId());
    }
    if (bits.test(9))
    {
        CD_INFO("\t-Remote: drive parameters may be modified via CAN and the drive will execute the command message. canId: %d.\n", can->getId());
    }
    else
    {
        CD_INFO("\t-Remote: drive is in local mode and will not execute the command message (only TML internal).");
    }
    if (bits.test(10))
    {
        CD_INFO("\t-Target reached. canId: %d.\n", can->getId());
    }
    else
    {
        CD_INFO("\t-Target not reached. canId: %d.\n", can->getId()); // improvised, not in manual, but reasonable
    }
    if (bits.test(11))
    {
        CD_INFO("\t-Internal Limit Active. canId: %d.\n", can->getId());
    }
    if (bits.test(14))
    {
        CD_INFO("\t-Last event set has ocurred. canId: %d.\n", can->getId());
    }
    else
    {
        CD_INFO("\t-No event set or the programmed event has not occurred yet. canId: %d.\n", can->getId());
    }
    if (bits.test(15))
    {
        CD_INFO("\t-Axis on. Power stage is enabled. Motor control is performed. canId: %d.\n", can->getId());
    }
    else
    {
        CD_INFO("\t-Axis off. Power stage is disabled. Motor control is not performed. canId: %d.\n", can->getId());
    }
    ////Much much more in Table 5.6
    if (bits.test(16))
    {
        CD_INFO("\t*Drive/motor initialization performed. canId: %d.\n", can->getId());
    }
    if (bits.test(17))
    {
        CD_INFO("\t*Position trigger 1 reached. canId: %d.\n", can->getId());
    }
    if (bits.test(18))
    {
        CD_INFO("\t*Position trigger 2 reached. canId: %d.\n", can->getId());
    }
    if (bits.test(19))
    {
        CD_INFO("\t*Position trigger 3 reached. canId: %d.\n", can->getId());
    }
    if (bits.test(20))
    {
        CD_INFO("\t*Position trigger 4 reached. canId: %d.\n", can->getId());
    }
    if (bits.test(21))
    {
        CD_INFO("\t*AUTORUN mode enabled. canId: %d.\n", can->getId());
    }
    if (bits.test(22))
    {
        CD_INFO("\t*Limit switch positive event / interrupt triggered. canId: %d.\n", can->getId());
    }
    if (bits.test(23))
    {
        CD_INFO("\t*Limit switch negative event / interrupt triggered. canId: %d.\n", can->getId());
    }
    if (bits.test(24))
    {
        CD_INFO("\t*Capture event/interrupt triggered. canId: %d.\n", can->getId());
    }
    if (bits.test(25))
    {
        CD_INFO("\t*Target command reached. canId: %d.\n", can->getId());
    }
    if (bits.test(26))
    {
        CD_INFO("\t*Motor I2t protection warning level reached. canId: %d.\n", can->getId());
    }
    if (bits.test(27))
    {
        CD_INFO("\t*Drive I2t protection warning level reached. canId: %d.\n", can->getId());
    }
    if (bits.test(28))
    {
        CD_INFO("\t*Gear ratio in electronic gearing mode reached. canId: %d.\n", can->getId());
    }
    if (bits.test(30))
    {
        CD_INFO("\t*Reference position in absolute electronic camming mode reached. canId: %d.\n", can->getId());
    }
    if (bits.test(31))
    {
        CD_INFO("\t*Drive/motor in fault status. canId: %d.\n", can->getId());
    }

    return true;
}

// -----------------------------------------------------------------------------

bool TechnosoftIpos::getControlModeRaw3()
{
    uint16_t data;

    if (!can->sdo()->upload("Motion Error Register", &data, 0x2000))
    {
        return false;
    }

    std::bitset<16> bits(data);

    if (bits.test(0))
    {
        CD_INFO("\t*CAN error. Set when CAN controller is in error mode. canId: %d.\n", can->getId());
    }
    if (bits.test(1))
    {
        CD_INFO("\t*Short-circuit. Set when protection is triggered. canId: %d.\n", can->getId());
    }
    if (bits.test(2))
    {
        CD_INFO("\t*Invalid setup data. Set when the EEPROM stored setup data is not valid or not present. canId: %d.\n", can->getId());
    }
    if (bits.test(3))
    {
        CD_INFO("\t*Control error (position/speed error too big). Set when protection is triggered. canId: %d.\n", can->getId());
    }
    if (bits.test(4))
    {
        CD_INFO("\t*Communication error. Set when protection is triggered. canId: %d.\n", can->getId());//true
    }
    if (bits.test(5))
    {
        CD_INFO("\t*Motor position wraps around. Set when protection is triggered. canId: %d.\n", can->getId());
    }
    if (bits.test(6))
    {
        CD_INFO("\t*Positive limit switch active. Set when LSP input is in active state. canId: %d.\n", can->getId());
    }
    if (bits.test(7))
    {
        CD_INFO("\t*Negative limit switch active. Set when LSN input is in active state. canId: %d.\n", can->getId());
    }
    if (bits.test(8))
    {
        CD_INFO("\t*Over current. Set when protection is triggered. canId: %d.\n", can->getId());
    }
    if (bits.test(9))
    {
        CD_INFO("\t*I2t protection. Set when protection is triggered. canId: %d.\n", can->getId());
    }
    if (bits.test(10))
    {
        CD_INFO("\t*Over temperature motor. Set when protection is triggered. canId: %d.\n", can->getId());
    }
    if (bits.test(11))
    {
        CD_INFO("\t*Over temperature drive. Set when protection is triggered. canId: %d.\n", can->getId());
    }
    if (bits.test(12))
    {
        CD_INFO("\t*Over-voltage. Set when protection is triggered. canId: %d.\n", can->getId());
    }
    if (bits.test(13))
    {
        CD_INFO("\t*Under-voltage. Set when protection is triggered. canId: %d.\n", can->getId());
    }
    if (bits.test(14))
    {
        CD_INFO("\t*Command error. This bit is set in several situations. They can be distinguished either by the associated emergency code, or in conjunction with other bits. canId: %d.\n", can->getId());
    }
    if (bits.test(15))
    {
        CD_INFO("\t*Drive disabled due to enable input. Set when enable input is on disable state. canId: %d.\n", can->getId());
    }

    return true;
}

// -----------------------------------------------------------------------------

bool TechnosoftIpos::getControlModeRaw4()
{
    uint16_t data;

    if (!can->sdo()->upload("Detailed Error Register", &data, 0x2002))
    {
        return false;
    }

    std::bitset<16> bits(data);

    if (bits.test(0))
    {
        CD_INFO("\t**The number of nested function calls exceeded the length of TML stack. Last function call was ignored. canId: %d.\n", can->getId());
    }
    if (bits.test(1))
    {
        CD_INFO("\t**A RET/RETI instruction was executed while no function/ISR was active. canId: %d.\n", can->getId());
    }
    if (bits.test(2))
    {
        CD_INFO("\t**A call to an inexistent homing routine was received. canId: %d.\n", can->getId());
    }
    if (bits.test(3))
    {
        CD_INFO("\t**A call to an inexistent function was received. canId: %d.\n", can->getId());
    }
    if (bits.test(4))
    {
        CD_INFO("\t**UPD instruction received while AXISON was executed. The UPD instruction was ingnored and it must be sent again when AXISON is completed. canId: %d.\n", can->getId());
    }
    if (bits.test(5))
    {
        CD_INFO("\t**Cancelable call instruction received while another cancelable function was active. canId: %d.\n", can->getId());
    }
    if (bits.test(6))
    {
        CD_INFO("\t**Positive software limit switch is active. canId: %d.\n", can->getId());
    }
    if (bits.test(7))
    {
        CD_INFO("\t**Negative software limit switch is active. canId: %d.\n", can->getId());
    }
    if (bits.test(8))
    {
        CD_INFO("\t**S-curve parameters caused and invalid profile. UPD instruction was ignored. canId: %d.\n", can->getId());
    }
    if (bits.test(9))
    {
        CD_INFO("\t**Update ignored for S-curve. canId: %d.\n", can->getId());
    }
    if (bits.test(10))
    {
        CD_INFO("\t**Encoder broken wire. canId: %d.\n", can->getId());
    }
    if (bits.test(11))
    {
        CD_INFO("\t**Motionless start failed. canId: %d.\n", can->getId());
    }
    if (bits.test(13))
    {
        CD_INFO("\t**Self check error. canId: %d.\n", can->getId());
    }

    return true;
}

// -----------------------------------------------------------------------------

bool TechnosoftIpos::getControlModesRaw(int * modes)
{
    CD_DEBUG("\n");
    return getControlModeRaw(0, &modes[0]);
}

// -----------------------------------------------------------------------------

bool TechnosoftIpos::getControlModesRaw(int n_joint, const int * joints, int * modes)
{
    CD_DEBUG("\n");
    return getControlModeRaw(joints[0], &modes[0]);
}

// -----------------------------------------------------------------------------

bool TechnosoftIpos::setControlModeRaw(int j, int mode)
{
    CD_DEBUG("(%d, %s)\n", j, yarp::os::Vocab::decode(mode).c_str());
    CHECK_JOINT(j);

    switch (mode)
    {
    case VOCAB_CM_POSITION:
        return can->sdo()->download<int8_t>("Modes of Operation", 1, 0x6060);
    case VOCAB_CM_VELOCITY:
        return can->sdo()->download<int8_t>("Modes of Operation", 3, 0x6060);
    case VOCAB_CM_CURRENT:
    case VOCAB_CM_TORQUE:
        vars.controlMode = mode;
        return can->sdo()->download<uint16_t>("External Reference Type", 1, 0x201D)
                && can->sdo()->download<int8_t>("Modes of Operation", -5, 0x6060)
                && can->rpdo1()->write<uint16_t>(0x001F);
    case VOCAB_CM_POSITION_DIRECT:
        return setPositionDirectModeRaw();
    default:
        CD_ERROR("Unsupported mode %s.\n", yarp::os::Vocab::decode(mode).c_str());
        return false;
    }
}

// -----------------------------------------------------------------------------

bool TechnosoftIpos::setControlModesRaw(int * modes)
{
    CD_DEBUG("\n");
    return setControlModeRaw(0, modes[0]);
}

// -----------------------------------------------------------------------------

bool TechnosoftIpos::setControlModesRaw(int n_joint, const int * joints, int * modes)
{
    CD_DEBUG("\n");
    return setControlModeRaw(joints[0], modes[0]);
}

// -----------------------------------------------------------------------------
