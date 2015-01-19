// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "BodyBot.hpp"

// ------------------ IPositionControl Related ----------------------------------------

bool teo::BodyBot::getAxes(int *axes) {
    *axes = drivers.size();
    return true;
}

// -----------------------------------------------------------------------------

bool teo::BodyBot::setPositionMode() {
    CD_INFO("\n");

    bool ok = true;
    for(int j=0; j<drivers.size(); j++)
    {
        ok &= this->setPositionMode(j);
    }
    return ok;
}

// -----------------------------------------------------------------------------

bool teo::BodyBot::positionMove(int j, double ref) {  // encExposed = ref;
    CD_INFO("(%d,%f)\n",j,ref);

    //-- Check index within range
    if ( ! this->indexWithinRange(j) ) return false;

    //*************************************************************
    uint8_t msg_position_target[]={0x23,0x7A,0x60,0x00,0x00,0x00,0x00,0x00}; // Position target

    int position = ref * (drivers[j]->getTr()) * 11.11112;  // Appply tr & convert units to encoder increments
    memcpy(msg_position_target+4,&position,4);

    if( ! drivers[j]->send( 0x600, 8, msg_position_target ) )
    {
        CD_ERROR("Could not send \"position target\" to canId: %d.\n",drivers[j]->getCanId());
        return false;
    }
    CD_SUCCESS("Sent \"position target\" to canId: %d.\n",drivers[j]->getCanId());
    //*************************************************************
    //uint8_t msg_start[]={0x1F,0x00}; // Start the movement with "Discrete motion profile (change set immediately = 0)".
    uint8_t msg_start[]={0x3F,0x00}; // Start the movement with "Continuous motion profile (change set immediately = 1)".

    if( ! drivers[j]->send( 0x200, 2, msg_start ) )
    {
        CD_ERROR("Could not send \"start position\" to canId: %d.\n",drivers[j]->getCanId());
        return false;
    }
    CD_SUCCESS("Sent \"start position\" to canId: %d.\n",drivers[j]->getCanId());
    //*************************************************************

    //-- Needed to send next. Sets "Do not assume target position" so later it accepts "Assume target position (update the new motion parameters)".
    //*************************************************************
    uint8_t msg_pos_reset[]={0x0F,0x00};  // Stop a position profile

    if( ! drivers[j]->send( 0x200, 2, msg_pos_reset) )
    {
        CD_ERROR("Could not send \"reset position\" to canId: %d.\n",drivers[j]->getCanId());
        return false;
    }
    CD_SUCCESS("Sent \"reset position\" to canId: %d.\n",drivers[j]->getCanId());
    //*************************************************************

    return true;
}

// -----------------------------------------------------------------------------

bool teo::BodyBot::positionMove(const double *refs) {  // encExposed = refs;
    CD_INFO("\n");

    //*************************************************************
    for(int j=0; j<drivers.size(); j++)
    {
        uint8_t msg_position_target[]={0x23,0x7A,0x60,0x00,0x00,0x00,0x00,0x00}; // Position target

        int position = refs[j] * (drivers[j]->getTr()) * 11.11112;  // Appply tr & convert units to encoder increments
        memcpy(msg_position_target+4,&position,4);

        if( ! drivers[j]->send( 0x600, 8, msg_position_target ) )
        {
            CD_ERROR("Could not send position target.\n");
            return false;
        }
        CD_SUCCESS("Sent \"position target\".\n");
    }
    //*************************************************************
    for(int j=0; j<drivers.size(); j++)
    {
        //uint8_t msg_start[]={0x1F,0x00}; // Start the movement with "Discrete motion profile (change set immediately = 0)".
        uint8_t msg_start[]={0x3F,0x00}; // Start the movement with "Continuous motion profile (change set immediately = 1)".

        if( ! drivers[j]->send( 0x200, 2, msg_start ) )
        {
            CD_ERROR("Could not send \"start position.\n");
            return false;
        }
        CD_SUCCESS("Sent \"start position\".\n");
    }
    //*************************************************************

    //-- Needed to send next. Sets "Do not assume target position" so later it accepts "Assume target position (update the new motion parameters)".
    //*************************************************************
    for(int j=0; j<drivers.size(); j++)
    {
        uint8_t msg_pos_reset[]={0x0F,0x00};  // Stop a position profile

        if( ! drivers[j]->send( 0x200, 2, msg_pos_reset) )
        {
            CD_ERROR("Could not send \"reset position\" to canId: %d.\n",drivers[j]->getCanId());
            return false;
        }
        CD_SUCCESS("Sent \"reset position\" to canId: %d.\n",drivers[j]->getCanId());
    }
    //*************************************************************

    return true;
}

// -----------------------------------------------------------------------------

bool teo::BodyBot::relativeMove(int j, double delta) {
    CD_INFO("(%d, %f)\n",j,delta);

    //-- Check index within range
    if ( ! this->indexWithinRange(j) ) return false;


    //*************************************************************
    uint8_t msg_position_target[]={0x23,0x7A,0x60,0x00,0x00,0x00,0x00,0x00}; // Position target

    int sendDelta = delta * (drivers[j]->getTr()) * 11.11112;  // Appply tr & convert units to encoder increments
    memcpy(msg_position_target+4,&sendDelta,4);

    if( ! drivers[j]->send( 0x600, 8, msg_position_target ) )
    {
        CD_ERROR("Could not send \"position target\".\n");
        return false;
    }
    CD_SUCCESS("Sent \"position target\".\n");
    //*************************************************************
    //uint8_t msg_start_rel[]={0x5F,0x00}; // Start the movement with "Discrete motion profile (change set immediately = 0)".
    uint8_t msg_start_rel[]={0x7F,0x00}; // Start the movement with "Continuous motion profile (change set immediately = 1)".


    if( ! drivers[j]->send( 0x200, 2, msg_start_rel ) )
    {
        CD_ERROR("Could not send \"start rel position.\n");
        return false;
    }
    CD_SUCCESS("Sent \"start rel position\".\n");
    //*************************************************************

    //-- Needed to send next. Sets "Do not assume target position" so later it accepts "Assume target position (update the new motion parameters)".
    //*************************************************************
    uint8_t msg_pos_reset[]={0x0F,0x00};  // Stop a position profile

    if( ! drivers[j]->send( 0x200, 2, msg_pos_reset) )
    {
        CD_ERROR("Could not send \"reset position\" to canId: %d.\n",drivers[j]->getCanId());
        return false;
    }
    CD_SUCCESS("Sent \"reset position\" to canId: %d.\n",drivers[j]->getCanId());
    //*************************************************************

    return true;
}

// -----------------------------------------------------------------------------

bool teo::BodyBot::relativeMove(const double *deltas) {  // encExposed = deltas + encExposed
    CD_INFO("\n");

    //*************************************************************
    for(int j=0; j<drivers.size(); j++)
    {
        uint8_t msg_position_target[]={0x23,0x7A,0x60,0x00,0x00,0x00,0x00,0x00}; // Position target

        int position = deltas[j] * (drivers[j]->getTr()) * 11.11112;  // Appply tr & convert units to encoder increments
        memcpy(msg_position_target+4,&position,4);

        if( ! drivers[j]->send( 0x600, 8, msg_position_target ) )
        {
            CD_ERROR("Could not send position target.\n");
            return false;
        }
        CD_SUCCESS("Sent \"position target\".\n");
    }
    //*************************************************************
    for(int j=0; j<drivers.size(); j++)
    {
        //uint8_t msg_start[]={0x5F,0x00}; // Start the movement with "Discrete motion profile (change set immediately = 0)".
        uint8_t msg_start[]={0x7F,0x00}; // Start the movement with "Continuous motion profile (change set immediately = 1)".

        if( ! drivers[j]->send( 0x200, 2, msg_start ) )
        {
            CD_ERROR("Could not send \"start position.\n");
            return false;
        }
        CD_SUCCESS("Sent \"start position\".\n");
    }
    //*************************************************************

    //-- Needed to send next. Sets "Do not assume target position" so later it accepts "Assume target position (update the new motion parameters)".
    //*************************************************************
    for(int j=0; j<drivers.size(); j++)
    {
        uint8_t msg_pos_reset[]={0x0F,0x00};  // Stop a position profile

        if( ! drivers[j]->send( 0x200, 2, msg_pos_reset) )
        {
            CD_ERROR("Could not send \"reset position\" to canId: %d.\n",drivers[j]->getCanId());
            return false;
        }
        CD_SUCCESS("Sent \"reset position\" to canId: %d.\n",drivers[j]->getCanId());
    }
    //*************************************************************

    return true;
}

// -----------------------------------------------------------------------------

bool teo::BodyBot::checkMotionDone(int j, bool *flag) {
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( ! this->indexWithinRange(j) ) return false;

    //*************************************************************
    uint8_t msgStatus[] = {0x40,0x41,0x60,0x00,0x00,0x00,0x00,0x00}; //2064: Memory position
    if( ! drivers[j]->send( 0x600, 8, msgStatus))
    {
        CD_ERROR("Could not send status query.\n");
        return false;
    }
    CD_SUCCESS("Sent \"msgStatus\" to canId: %d.\n",drivers[j]->getCanId());

    //* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
    Time::delay(DELAY);  //-- Wait for read update. Could implement semaphore waiting for specific message...
    //* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *

    *flag = drivers[j]->targetReached;

    return true;
}

// -----------------------------------------------------------------------------

bool teo::BodyBot::checkMotionDone(bool *flag) {
    CD_INFO("\n");

    bool ok = true;
    for(int j=0; j<drivers.size(); j++)
    {
        ok &= this->checkMotionDone(j,&flag[j]);
    }
    return ok;
}

// -----------------------------------------------------------------------------

bool teo::BodyBot::setRefSpeed(int j, double sp) {
    CD_INFO("(%d, %f)\n",j,sp);

    //-- Check index within range
    if ( ! this->indexWithinRange(j) ) return false;

    //*************************************************************

    uint8_t msg_posmode_speed[]={0x23,0x81,0x60,0x00,0x00,0x00,0x00,0x00};

    int sendRefSpeed = sp * (drivers[j]->getTr()) * 11.11112;  // Appply tr & convert units to encoder increments
    memcpy(msg_posmode_speed+4,&sendRefSpeed,4);

    if( ! drivers[j]->send( 0x600, 8, msg_posmode_speed) )
    {
        CD_ERROR("Could not send \"posmode_speed\" to canId: %d.\n",drivers[j]->getCanId());
        return false;
    }
    CD_SUCCESS("Sent \"posmode_speed\" to canId: %d.\n",drivers[j]->getCanId());
    //*************************************************************

    //-- Store new value locally as we can not retrieve it from the driver for now.
    drivers[j]->setRefSpeed( sp );

    return true;
}

// -----------------------------------------------------------------------------

bool teo::BodyBot::setRefSpeeds(const double *spds) {
    CD_INFO("\n");

    bool ok = true;
    for(unsigned int i=0;i<drivers.size();i++)
        ok &= setRefSpeed(i,spds[i]);
    return ok;
}

// -----------------------------------------------------------------------------

bool teo::BodyBot::setRefAcceleration(int j, double acc) {
    CD_INFO("(%d, %f)\n",j,acc);

    //-- Check index within range
    if ( ! this->indexWithinRange(j) ) return false;

    //*************************************************************
    uint8_t msg_posmode_acc[]={0x23,0x83,0x60,0x00,0x00,0x00,0x00,0x00};

    int sendRefAcc = acc * (drivers[j]->getTr()) * 11.11112;  // Appply tr & convert units to encoder increments
    memcpy(msg_posmode_acc+4,&sendRefAcc,4);

    if( ! drivers[j]->send( 0x600, 8, msg_posmode_acc) )
    {
        CD_ERROR("Could not send \"posmode_acc\" to canId: %d.\n",drivers[j]->getCanId());
        return false;
    }
    CD_SUCCESS("Sent \"posmode_acc\" to canId: %d.\n",drivers[j]->getCanId());
    //*************************************************************

    //-- Store new value locally as we can not retrieve it from the driver for now.
    drivers[j]->setRefAcceleration( acc );

    return true;
}

// -----------------------------------------------------------------------------

bool teo::BodyBot::setRefAccelerations(const double *accs) {
    CD_INFO("\n");

    bool ok = true;
    for(unsigned int i=0;i<drivers.size();i++)
        ok &= setRefAcceleration(i,accs[i]);
    return ok;
}

// -----------------------------------------------------------------------------

bool teo::BodyBot::getRefSpeed(int j, double *ref) {
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( ! this->indexWithinRange(j) ) return false;

    *ref = drivers[j]->getRefSpeed();

    return true;
}

// -----------------------------------------------------------------------------

bool teo::BodyBot::getRefSpeeds(double *spds) {
    CD_INFO("\n");

    bool ok = true;
    for(unsigned int i=0;i<drivers.size();i++)
        ok &= getRefSpeed(i,&spds[i]);
    return ok;
}

// -----------------------------------------------------------------------------

bool teo::BodyBot::getRefAcceleration(int j, double *acc) {
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( ! this->indexWithinRange(j) ) return false;

    *acc = drivers[j]->getRefAcceleration();

    return true;
}

// -----------------------------------------------------------------------------

bool teo::BodyBot::getRefAccelerations(double *accs) {
    CD_INFO("\n");

    bool ok = true;
    for(unsigned int i=0;i<drivers.size();i++)
        ok &= getRefAcceleration(i,&accs[i]);
    return ok;
}

// -----------------------------------------------------------------------------

bool teo::BodyBot::stop(int j) {
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( ! this->indexWithinRange(j) ) return false;

    CD_WARNING("Not implemented yet.\n");

    return true;
}

// -----------------------------------------------------------------------------

bool teo::BodyBot::stop() {
    CD_INFO("\n");

    bool ok = true;
    for(unsigned int i=0;i<drivers.size();i++)
        ok &= stop(i);
    return ok;
}

// -----------------------------------------------------------------------------

