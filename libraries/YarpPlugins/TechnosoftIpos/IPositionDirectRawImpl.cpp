// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TechnosoftIpos.hpp"

// ------------------ IPositionDirect Related ----------------------------------

bool roboticslab::TechnosoftIpos::setPositionRaw(int j, double ref)
{
    CD_INFO("(%d,%f)\n",j,ref);

    //-- Check index within range
    if ( j != 0 ) return false;

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
    if (std::abs(ref - lastPtRef) > maxPtDistance)
    {
        CD_WARNING("Max velocity exceeded, clipping travelled distance.\n");
        ref = lastPtRef + maxPtDistance * (ref >= lastPtRef ? 1 : -1);
        CD_INFO("New ref: %f.\n", ref);
    }
    uint8_t msg_ptPoint[8];
    int32_t position = ref * this->tr * (encoderPulses / 360.0); // Apply tr & convert units to encoder increments
    memcpy(msg_ptPoint+0,&position,4);
    memcpy(msg_ptPoint+4,&ptModeMs,2);
    uint8_t ic = (++pvtPointCounter) << 1;
    memcpy(msg_ptPoint+7,&ic,1);
    yarp::os::Time::delay(0.001); // 0.01 okay for 1 arm (6 motors), had to reduce to 0.0075 for two arms (12 motors)
    if ( ! send(0x400,8,msg_ptPoint) )
    {
        CD_ERROR("msg_ptPoint in %d",canId);
        return false;
    }
    CD_SUCCESS("Sent to canId %d: pos %f, time %d, ic %d.\n",canId,ref,ptModeMs,ic);
    lastPtRef = ref;

    pvtThread->updateTarget(ref);

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
