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

bool roboticslab::TechnosoftIpos::setTrajectoryRaw(int j, double ref)
{
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
    int32_t position = ref * this->tr * (encoderPulses / 360.0);  // Appply tr & convert units to encoder increments
    memcpy(msg_ptPoint+0,&position,4);
    memcpy(msg_ptPoint+4,&(this->ptModeMs),2);
    uint8_t ic = (ptPointCounter+1)*2;  //-- *2 because only 7 bits, so only half the buffer is usable
    memcpy(msg_ptPoint+7,&ic,1);

    yarp::os::Time::delay(0.001); // 0.01 okay for 1 arm (6 motors), had to reduce to 0.0075 for two arms (12 motors)
    ptBuffer.wait();
    if ( ! send(0x400,8,msg_ptPoint) )
    {
        CD_ERROR("msg_ptPoint in %d",canId);
        return false;
    }
    CD_SUCCESS("Sent to canId %d: pos %f, time %d, ic %d.\n",canId,ref,ptModeMs,ic);
    ptBuffer.post();

    ptPointCounter++;


    //*************************************************************
    uint8_t startPT[]= {0x1F,0x00};
    //Send start

    if( ptPointCounter == 7 )    //-- Put max buffer length here (usually 7).
    {

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
