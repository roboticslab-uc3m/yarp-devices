// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TechnosoftIpos.hpp"

// ------------------- IControlLimitsRaw Related ------------------------------------

bool roboticslab::TechnosoftIpos::setLimitsRaw(int axis, double min, double max)
{
    CD_INFO("(%d,%f,%f)\n",axis,min,max);

    //-- Check index within range
    if ( axis != 0 ) return false;

    bool ok = true;
    ok &= setMinLimitRaw(min);
    ok &= setMaxLimitRaw(max);

    return ok;
}

// -----------------------------------------------------------------------------
bool roboticslab::TechnosoftIpos::setMinLimitRaw(double min)
{
    uint8_t msg_position_min[]= {0x23,0x7D,0x60,0x01,0x00,0x00,0x00,0x00}; // 0x01 is subindex 1, Manual 607Dh: Software position limit
    if(this->tr < 0) msg_position_min[3] = 0x02;

    int sendMin = min * this->tr * (encoderPulses / 360.0);  // Appply tr & convert units to encoder increments
    memcpy(msg_position_min+4,&sendMin,4);

    if( ! send( 0x600, 8, msg_position_min ) )
    {
        CD_ERROR("Could not send position min. %s\n", msgToStr(0x600, 8, msg_position_min).c_str() );
        return false;
    }
    CD_SUCCESS("Sent \"position min\". %s\n", msgToStr(0x600, 8, msg_position_min).c_str() );

    if (!sdoSemaphore->await(msg_position_min))
    {
        CD_ERROR("Did not receive position min ack. %s\n", msgToStr(0x600, 8, msg_position_min).c_str());
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------------
bool roboticslab::TechnosoftIpos::setMaxLimitRaw(double max)
{
    uint8_t msg_position_max[]= {0x23,0x7D,0x60,0x02,0x00,0x00,0x00,0x00}; // 0x02 is subindex 2, Manual 607Dh: Software position limit
    if(this->tr < 0) msg_position_max[3] = 0x01;

    int sendMax = max * this->tr * (encoderPulses / 360.0);  // Appply tr & convert units to encoder increments
    memcpy(msg_position_max+4,&sendMax,4);

    if( ! send( 0x600, 8, msg_position_max ) )
    {
        CD_ERROR("Could not send position max. %s\n", msgToStr(0x600, 8, msg_position_max).c_str() );
        return false;
    }
    CD_SUCCESS("Sent \"position max\". %s\n", msgToStr(0x600, 8, msg_position_max).c_str() );

    if (!sdoSemaphore->await(msg_position_max))
    {
        CD_ERROR("Did not receive position max ack. %s\n", msgToStr(0x600, 8, msg_position_max).c_str());
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::TechnosoftIpos::getLimitsRaw(int axis, double *min, double *max)
{
    CD_INFO("(%d)\n",axis);

    //-- Check index within range
    if( axis != 0 ) return false;

    bool ok = true;
    ok &= getMinLimitRaw(min);
    ok &= getMaxLimitRaw(max);

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::TechnosoftIpos::getMinLimitRaw(double *min)
{
    uint8_t msg_position_min[]= {0x40,0x7D,0x60,0x01,0x00,0x00,0x00,0x00}; // 0x01 is subindex 1, Manual 607Dh: Software position limit
    if(this->tr < 0) msg_position_min[3] = 0x02;

    if( ! send( 0x600, 8, msg_position_min ) )
    {
        CD_ERROR("Could not send position min query. %s\n", msgToStr(0x600, 8, msg_position_min).c_str() );
        return false;
    }
    CD_SUCCESS("Sent \"position min\" query. %s\n", msgToStr(0x600, 8, msg_position_min).c_str() );

    if (!sdoSemaphore->await(msg_position_min))
    {
        CD_ERROR("Did not receive position min response. %s\n", msgToStr(0x600, 8, msg_position_min).c_str());
        return false;
    }

    int32_t got;
    std::memcpy(&got, msg_position_min + 4, 4);
    *min = got / (tr * encoderPulses / 360.0);

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::TechnosoftIpos::getMaxLimitRaw(double *max)
{
    uint8_t msg_position_max[]= {0x40,0x7D,0x60,0x02,0x00,0x00,0x00,0x00}; // 0x02 is subindex 2, Manual 607Dh: Software position limit
    if(this->tr < 0) msg_position_max[3] = 0x01;

    if( ! send( 0x600, 8, msg_position_max ) )
    {
        CD_ERROR("Could not send position max query. %s\n", msgToStr(0x600, 8, msg_position_max).c_str() );
        return false;
    }
    CD_SUCCESS("Sent \"position max\" query. %s\n", msgToStr(0x600, 8, msg_position_max).c_str() );

    if (!sdoSemaphore->await(msg_position_max))
    {
        CD_ERROR("Did not receive position max response. %s\n", msgToStr(0x600, 8, msg_position_max).c_str());
        return false;
    }

    int32_t got;
    std::memcpy(&got, msg_position_max + 4, 4);
    *max = got / (tr * encoderPulses / 360.0);

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::TechnosoftIpos::setVelLimitsRaw(int axis, double min, double max)
{
    CD_INFO("(%d,%f,%f)\n",axis,min,max);

    //-- Check index within range
    if ( axis != 0 ) return false;

    //-- Update the limits that have been locally stored.
    this->maxVel = max;

    if (min != -max)
    {
        CD_WARNING("Minimum value not equal to negative maximum value.\n");
    }

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::TechnosoftIpos::getVelLimitsRaw(int axis, double *min, double *max)
{
    CD_INFO("(%d)\n",axis);

    //-- Check index within range
    if( axis != 0 ) return false;

    //-- Get the limits that have been locally stored.
    *min = -this->maxVel;
    *max = this->maxVel;

    return true;
}

// -----------------------------------------------------------------------------
