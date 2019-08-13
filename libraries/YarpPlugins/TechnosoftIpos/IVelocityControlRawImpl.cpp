// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TechnosoftIpos.hpp"

#include <cmath>
#include <cstring>

// ######################## IVelocityControlRaw Related #############################

bool roboticslab::TechnosoftIpos::velocityMoveRaw(int j, double sp)
{
    CD_DEBUG("(%d),(%f)\n",j,sp);

    //-- Check index within range
    if ( j != 0 ) return false;

    if ( sp > maxVel )
    {
        CD_WARNING("Requested speed exceeds maximum velocity (%f).\n", maxVel);
        return false;
    }

    //*************************************************************
    uint8_t msg_vel[]= {0x23,0xFF,0x60,0x00,0x00,0x00,0x00,0x00}; // Velocity target

    //uint16_t sendRefSpeed = sp * this->tr * 0.01138;  // Appply tr & convert units to encoder increments
    //memcpy(msg_posmode_speed+6,&sendRefSpeed,2);
    //float sendRefSpeed = sp * this->tr / 22.5;  // Apply tr & convert units to encoder increments
    //int32_t sendRefSpeedFormated = roundf(sendRefSpeed * 65536);  // 65536 = 2^16

    //-- 0.01138 = ( 4 * 1024 pulse / 360 deg ) * (0.001 s / sample)   // deg/s -> pulse/sample  = UI (vel)
    // encoderPulses: value encompasses the pulses-per-slot factor (usually 4) and number of total slots of the encoder (currently: 4 * 1024)
    double sendRefSpeed = sp * this->tr * (encoderPulses / 360.0) * 0.001;
    int16_t sendRefSpeedInteger;
    uint16_t sendRefSpeedFractional;
    encodeFixedPoint(sendRefSpeed, &sendRefSpeedInteger, &sendRefSpeedFractional);
    memcpy(msg_vel + 4, &sendRefSpeedFractional, 2);
    memcpy(msg_vel + 6, &sendRefSpeedInteger, 2);

    if( ! send(0x600, 8, msg_vel))
    {
        CD_ERROR("Sent \"velocity target\". %s\n", msgToStr(0x600, 8, msg_vel).c_str() );
        return false;
    }
    CD_SUCCESS("Sent \"velocity target\". %s\n", msgToStr(0x600, 8, msg_vel).c_str() );

    if (!sdoSemaphore->await(msg_vel))
    {
        CD_ERROR("Did not receive \"velocity target\" ack. %s\n", msgToStr(0x600, 8, msg_vel).c_str());
        return false;
    }

    // -- Save the last reference speed (double sp) for single joint (int j)
    refVelocitySemaphore.wait();
    refVelocity = sp;
    refVelocitySemaphore.post();

    return true;
}

// ----------------------------------------------------------------------------------

bool roboticslab::TechnosoftIpos::velocityMoveRaw(const double *sp)
{
    CD_ERROR("Missing implementation\n");
    return false;
}

// ######################## IVelocityControlRaw2 Related ########################

bool roboticslab::TechnosoftIpos::velocityMoveRaw(const int n_joint, const int *joints, const double *spds)
{
    CD_DEBUG("Missing implementation\n");
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::TechnosoftIpos::getRefVelocityRaw(const int joint, double *vel)
{
    CD_DEBUG("(%d)\n",joint);

    //-- Check index within range
    if ( joint != 0 ) return false;

    //*************************************************************
    uint8_t msg_vel[]= {0x40,0xFF,0x60,0x00,0x00,0x00,0x00,0x00}; // Velocity target

    if( ! send(0x600, 8, msg_vel))
    {
        CD_ERROR("Sent \"velocity target\" query. %s\n", msgToStr(0x600, 8, msg_vel).c_str() );
        return false;
    }
    CD_SUCCESS("Sent \"velocity target\" query. %s\n", msgToStr(0x600, 8, msg_vel).c_str() );
    //*************************************************************

    if (!sdoSemaphore->await(msg_vel))
    {
        CD_ERROR("Did not receive \"velocity target\" response. %s\n", msgToStr(0x600, 8, msg_vel).c_str());
        return false;
    }

    int16_t gotInteger;
    uint16_t gotFractional;

    std::memcpy(&gotFractional, msg_vel + 4, 2);
    std::memcpy(&gotInteger, msg_vel + 6, 2);

    double val = decodeFixedPoint(gotInteger, gotFractional);
    *vel = val / (tr * (encoderPulses / 360.0) * 0.001);

    return true;
}

// ------------------------------------------------------------------------------

bool roboticslab::TechnosoftIpos::getRefVelocitiesRaw(double *vels)
{
    CD_WARNING("Missing implementation\n");
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::TechnosoftIpos::getRefVelocitiesRaw(const int n_joint, const int *joints, double *vels)
{
    CD_WARNING("Missing implementation\n");
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::TechnosoftIpos::setRefAccelerationsRaw(const int n_joint, const int *joints, const double *accs)
{
    CD_WARNING("Missing implementation\n");
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::TechnosoftIpos::getRefAccelerationsRaw(const int n_joint, const int *joints, double *accs)
{
    CD_WARNING("Missing implementation\n");
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::TechnosoftIpos::stopRaw(const int n_joint, const int *joints)
{
    CD_WARNING("Missing implementation\n");
    return true;
}

// -----------------------------------------------------------------------------
