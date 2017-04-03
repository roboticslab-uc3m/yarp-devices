// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TechnosoftIpos.hpp"

// ######################## IVelocityControlRaw Related #############################

bool teo::TechnosoftIpos::setVelocityModeRaw()
{
    CD_ERROR("Missing implementation\n");
    return false;
}

// ----------------------------------------------------------------------------------

bool teo::TechnosoftIpos::velocityMoveRaw(int j, double sp)
{
    CD_DEBUG("(%d),(%f)\n",j,sp);

    //-- Check index within range
    if ( j != 0 ) return false;

    //*************************************************************
    uint8_t msg_vel[]= {0x23,0xFF,0x60,0x00,0x00,0x00,0x00,0x00}; // Velocity target

    //uint16_t sendRefSpeed = sp * this->tr * 0.01138;  // Appply tr & convert units to encoder increments
    //memcpy(msg_posmode_speed+6,&sendRefSpeed,2);
    //float sendRefSpeed = sp * this->tr / 22.5;  // Apply tr & convert units to encoder increments
    //int32_t sendRefSpeedFormated = roundf(sendRefSpeed * 65536);  // 65536 = 2^16

    //-- 65536 for FIXED32
    //-- 0.01138 = ( 4 * 1024 pulse / 360 deg ) * (0.001 s / sample)   // deg/s -> pulse/sample  = UI (vel)
    int32_t sendRefSpeedFormated = roundf( sp * this->tr * 745.8 ); //-- 65536 * 0.01138 = 745.8
    memcpy(msg_vel+4,&sendRefSpeedFormated,4);

    if( ! send(0x600, 8, msg_vel))
    {
        CD_ERROR("Sent \"velocity target\". %s\n", msgToStr(0x600, 8, msg_vel).c_str() );
        return false;
    }
    CD_SUCCESS("Sent \"velocity target\". %s\n", msgToStr(0x600, 8, msg_vel).c_str() );
    //*************************************************************

    // -- Save the last reference speed (double sp) for single joint (int j)
    refVelocitySemaphore.wait();
    refVelocity = sp;
    refVelocitySemaphore.post();

    return true;
}

// ----------------------------------------------------------------------------------

bool teo::TechnosoftIpos::velocityMoveRaw(const double *sp)
{
    CD_ERROR("Missing implementation\n");
    return false;
}

// ######################## IVelocityControlRaw2 Related ########################

bool teo::TechnosoftIpos::velocityMoveRaw(const int n_joint, const int *joints, const double *spds)
{
    CD_DEBUG("Missing implementation\n");
    return true;
}

// -----------------------------------------------------------------------------

bool teo::TechnosoftIpos::getRefVelocityRaw(const int joint, double *vel)
{

    // -- Get the last reference speed set by velocityMove for single joint (saved in double vector)
    refVelocitySemaphore.wait();
    CD_DEBUG("%d, %f\n",joint, refVelocity);
    *vel = refVelocity;
    refVelocitySemaphore.post();

    return true;
}

// ------------------------------------------------------------------------------

bool teo::TechnosoftIpos::getRefVelocitiesRaw(double *vels)
{
    CD_WARNING("Missing implementation\n");
    return true;
}

// -----------------------------------------------------------------------------

bool teo::TechnosoftIpos::getRefVelocitiesRaw(const int n_joint, const int *joints, double *vels)
{
    CD_WARNING("Missing implementation\n");
    return true;
}

// -----------------------------------------------------------------------------

bool teo::TechnosoftIpos::setRefAccelerationsRaw(const int n_joint, const int *joints, const double *accs)
{
    CD_WARNING("Missing implementation\n");
    return true;
}

// -----------------------------------------------------------------------------

bool teo::TechnosoftIpos::getRefAccelerationsRaw(const int n_joint, const int *joints, double *accs)
{
    CD_WARNING("Missing implementation\n");
    return true;
}

// -----------------------------------------------------------------------------

bool teo::TechnosoftIpos::stopRaw(const int n_joint, const int *joints)
{
    CD_WARNING("Missing implementation\n");
    return true;
}

// -----------------------------------------------------------------------------

bool teo::TechnosoftIpos::setVelPidRaw(int j, const yarp::dev::Pid &pid)
{
    CD_WARNING("Missing implementation\n");
    return true;
}

// -----------------------------------------------------------------------------

bool teo::TechnosoftIpos::setVelPidsRaw(const yarp::dev::Pid *pids)
{
    CD_WARNING("Missing implementation\n");
    return true;
}

// -----------------------------------------------------------------------------

bool teo::TechnosoftIpos::getVelPidRaw(int j, yarp::dev::Pid *pid)
{
    CD_WARNING("Missing implementation\n");
    return true;
}

// -----------------------------------------------------------------------------

bool teo::TechnosoftIpos::getVelPidsRaw(yarp::dev::Pid *pids)
{
    CD_WARNING("Missing implementation\n");
    return true;
}

