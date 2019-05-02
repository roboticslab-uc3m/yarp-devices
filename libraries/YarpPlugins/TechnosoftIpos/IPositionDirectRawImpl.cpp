// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TechnosoftIpos.hpp"

// ------------------ IPositionDirect Related ----------------------------------

bool roboticslab::TechnosoftIpos::setPositionRaw(int j, double ref)
{
    CD_INFO("(%d,%f)\n",j,ref);

    //-- Check index within range
    if ( j != 0 ) return false;

    //*************************************************************
    uint8_t msg_ref_position[]= {0x23,0x1C,0x20,0x00,0x00,0x00,0x00,0x00}; // put 23 because it is a target

    int position = ref * this->tr * (encoderPulses / 360.0);  // Appply tr & convert units to encoder increments
    memcpy(msg_ref_position+4,&position,4);

    if(! send(0x600, 8, msg_ref_position) )
    {
        CD_ERROR("Could not send ref_position. %s\n", msgToStr(0x600, 8, msg_ref_position).c_str() );
        return false;
    }
    CD_SUCCESS("Sent ref_position. %s\n", msgToStr(0x600, 8, msg_ref_position).c_str() );
    //*************************************************************

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::TechnosoftIpos::setPositionsRaw(const int n_joint, const int *joints, const double *refs)
{
    CD_WARNING("Not implemented yet.\n");
    return false;
}

// -----------------------------------------------------------------------------

bool roboticslab::TechnosoftIpos::setPositionsRaw(const double *refs)
{
    CD_WARNING("Not implemented yet.\n");
    return true;
}

// -----------------------------------------------------------------------------
