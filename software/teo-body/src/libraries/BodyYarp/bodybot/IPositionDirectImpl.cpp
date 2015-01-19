// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "BodyBot.hpp"

// ------------------ IPositionDirect Related ----------------------------------

bool teo::BodyBot::setPositionDirectMode() {
    CD_INFO("\n");

    bool ok = true;
    for(unsigned int i=0; i < drivers.size(); i++)
        ok &= drivers[i]->setPositionDirectModeRaw();  // No existing single mode.
    return ok;
}


// -----------------------------------------------------------------------------

bool teo::BodyBot::setPosition(int j, double ref) {
    CD_INFO("\n");

    return true;
}

// -----------------------------------------------------------------------------

bool teo::BodyBot::setPositions(const int n_joint, const int *joints, double *refs) {
    CD_INFO("n_joint:%d, drivers.size():" CD_SIZE_T "\n",n_joint,drivers.size());

    for(int j=0;j<drivers.size();j++)
    {
        CD_INFO("j:%d ref:%f\n",j,refs[j]);
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
        int32_t position = refs[j] * (drivers[j]->getTr()) * 11.11112;  // Appply tr & convert units to encoder increments
        memcpy(msg_ptPoint+0,&position,4);
        memcpy(msg_ptPoint+4,&(this->ptModeMs),2);
        uint8_t ic = (drivers[j]->ptPointCounter+1)*2;  //-- *2 because only 7 bits, so only half the buffer is usable
        memcpy(msg_ptPoint+7,&ic,1);

        yarp::os::Time::delay(0.001); // 0.01 okay for 1 arm (6 motors), had to reduce to 0.0075 for two arms (12 motors)
        drivers[j]->ptBuffer.wait();
        if ( ! drivers[j]->send(0x400,8,msg_ptPoint) ) {
            CD_ERROR("msg_ptPoint in %d",j);
            return false;
        }
        CD_SUCCESS("Sent to canId %d: pos %f, time %d, ic %d.\n",drivers[j]->getCanId(),refs[j],ptModeMs,ic);
        drivers[j]->ptBuffer.post();

        drivers[j]->ptPointCounter++;

    }
    //*************************************************************
    uint8_t startPT[]={0x1F,0x00};
    //Send start
    for(int j=0;j<drivers.size();j++)
    {
        if( drivers[j]->ptPointCounter == 7 ) {  //-- Put max buffer length here (usually 7).

            if( ! drivers[j]->send(0x200,2,startPT) )
            {
                CD_ERROR("Could not send \"startPT\" to canId: %d.\n",drivers[j]->getCanId());
                return false;
            }
            CD_SUCCESS("Sent \"startPT\" to canId: %d.\n",drivers[j]->getCanId());
        }
    }
    //*************************************************************
    /*uint8_t stopPT[]={0x0F,0x00};

    for(int j=0;j<drivers.size();j++)
    {
        CD_INFO("Wait for canId %d...\n",drivers[j]->getCanId());

        while( ! drivers[j]->ptMovementDone );

        yarp::os::Time::delay(0.01);
        if( ! drivers[j]->send(0x200,2,stopPT) )
        {
            CD_ERROR("Could not send \"stopPT\" to canId: %d.\n",drivers[j]->getCanId());
            return false;
        }
        CD_SUCCESS("Sent \"stopPT\" to canId: %d.\n",drivers[j]->getCanId());

    }*/
    //*************************************************************

    return true;
}

// -----------------------------------------------------------------------------

bool teo::BodyBot::setPositions(const double *refs) {
    return true;
}

// -----------------------------------------------------------------------------
