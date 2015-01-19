// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "MotorIpos.hpp"

// ------------------- ITorqueControl Related ------------------------------------

bool teo::MotorIpos::setTorqueMode() {
    CD_INFO("\n");

    bool ok = true;
    for(int j=0; j<drivers.size(); j++)
    {
        ok &= this->setTorqueMode(j);
    }
    return ok;
}

// -----------------------------------------------------------------------------

bool teo::MotorIpos::getRefTorques(double *t){
    CD_INFO("\n");

    CD_WARNING("Not implemented yet.\n");

    return true;
}

// -----------------------------------------------------------------------------

bool teo::MotorIpos::getRefTorque(int j, double *t) {
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( ! this->indexWithinRange(j) ) return false;

    CD_WARNING("Not implemented yet.\n");

    return true;
}

// -----------------------------------------------------------------------------

bool teo::MotorIpos::setRefTorques(const double *t) {
    CD_INFO("\n");

    CD_WARNING("Not implemented yet.\n");

    return true;
}


// -----------------------------------------------------------------------------

bool teo::MotorIpos::setRefTorque(int j, double t) {
    CD_INFO("(%d,%f)\n",j,t);

    //-- Check index within range
    if ( ! this->indexWithinRange(j) ) return false;

    //*************************************************************
    uint8_t msg_ref_torque[]={0x23,0x1C,0x20,0x00,0x00,0x00,0x00,0x00}; // put 23 because it is a target

    int sendRefTorque = t * (65520.0/20.0);
    //memcpy(msg_ref_torque+4,&sendRefTorque,4);  // was +6 not +4, but +6 seems terrible with ,4!
    memcpy(msg_ref_torque+6,&sendRefTorque,2);

    if(! drivers[j]->send(0x600, 8, msg_ref_torque) )
    {
        CD_ERROR("Could not send refTorque to canId: %d.\n",drivers[j]->getCanId());
        return false;
    }
    CD_SUCCESS("Sent refTorque.\n");
    //*************************************************************

    return true;
}

// -----------------------------------------------------------------------------

bool teo::MotorIpos::getBemfParam(int j, double *bemf) {
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( ! this->indexWithinRange(j) ) return false;

    CD_WARNING("Not implemented yet.\n");

    return true;
}

// -----------------------------------------------------------------------------

bool teo::MotorIpos::setBemfParam(int j, double bemf) {
    CD_INFO("(%d,%f)\n",j,bemf);

    //-- Check index within range
    if ( ! this->indexWithinRange(j) ) return false;

    CD_WARNING("Not implemented yet.\n");

    return true;
}

// -----------------------------------------------------------------------------

bool teo::MotorIpos::setTorquePid(int j, const Pid &pid) {
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( ! this->indexWithinRange(j) ) return false;

    CD_WARNING("Not implemented yet.\n");

    return true;
}

// -----------------------------------------------------------------------------

bool teo::MotorIpos::getTorque(int j, double *t) {
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( ! this->indexWithinRange(j) ) return false;

    uint8_t msg_read[]={0x40,0x7E,0x20,0x00}; // Query current. Ok only 4.

    //memcpy(msg_read,,4);
    if( ! drivers[j]->send(0x600,4,msg_read) ) {
        CD_ERROR("Failed to send part of read failed.\n");
        return false;
    }
    //display(message);
    Time::delay(DELAY);
    //purge(canNode);
    //CAN::receive(canNode);
    struct can_msg reply;
    return false; //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    ///////////////////////////if( ! canDevice.read_timeout(&reply,500) ){
        //////////////////////////////CD_WARNING("Read timeout!\n");
    ///////////////////////////////////////////////}
    //display(reply);
    //int got;
    //memcpy(&got,reply.data+4,4);
    //*current = got/(11.11112*devices[idx].tr);
    //int got;START LOOP
    //memcpy(&got,reply.data+4,4);
    //*current = got;
    int16_t got;
    //lastCurrent = got;
    //memcpy(&got,reply.data+4,2);
    //*current = - got / 4268.4 ;
    memcpy(&got,reply.data+4,4);
    * t = got * (2.0 * 10.0) / 65520.0;

    return true;
}

// -----------------------------------------------------------------------------

bool teo::MotorIpos::getTorques(double *t) {
    CD_INFO("\n");

    CD_WARNING("Not implemented yet.\n");

    return true;
}

// -----------------------------------------------------------------------------

bool teo::MotorIpos::getTorqueRange(int j, double *min, double *max) {
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( ! this->indexWithinRange(j) ) return false;

    CD_WARNING("Not implemented yet.\n");

    return true;
}

// -----------------------------------------------------------------------------

bool teo::MotorIpos::getTorqueRanges(double *min, double *max) {
    CD_INFO("\n");

    CD_WARNING("Not implemented yet.\n");

    return true;
}

// -----------------------------------------------------------------------------

bool teo::MotorIpos::setTorquePids(const Pid *pids) {
    CD_INFO("\n");

    CD_WARNING("Not implemented yet.\n");

    return true;
}

// -----------------------------------------------------------------------------

bool teo::MotorIpos::setTorqueErrorLimit(int j, double limit) {
    CD_INFO("(%d,%f)\n",j,limit);

    //-- Check index within range
    if ( ! this->indexWithinRange(j) ) return false;

    CD_WARNING("Not implemented yet.\n");

    return true;
}

// -----------------------------------------------------------------------------

bool teo::MotorIpos::setTorqueErrorLimits(const double *limits) {
    CD_INFO("\n");

    CD_WARNING("Not implemented yet.\n");

    return true;
}

// -----------------------------------------------------------------------------

bool teo::MotorIpos::getTorqueError(int j, double *err) {
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( ! this->indexWithinRange(j) ) return false;

    CD_WARNING("Not implemented yet.\n");

    return true;
}

// -----------------------------------------------------------------------------

bool teo::MotorIpos::getTorqueErrors(double *errs) {
    CD_INFO("\n");

    CD_WARNING("Not implemented yet.\n");

    return true;
}

// -----------------------------------------------------------------------------

bool teo::MotorIpos::getTorquePidOutput(int j, double *out) {
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( ! this->indexWithinRange(j) ) return false;

    CD_WARNING("Not implemented yet.\n");

    return true;
}

// -----------------------------------------------------------------------------

bool teo::MotorIpos::getTorquePidOutputs(double *outs) {
    CD_INFO("\n");

    CD_WARNING("Not implemented yet.\n");

    return true;
}

// -----------------------------------------------------------------------------

bool teo::MotorIpos::getTorquePid(int j, Pid *pid) {
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( ! this->indexWithinRange(j) ) return false;

    CD_WARNING("Not implemented yet.\n");

    return true;
}

// -----------------------------------------------------------------------------

bool teo::MotorIpos::getTorquePids(Pid *pids){
    CD_INFO("\n");

    CD_WARNING("Not implemented yet.\n");

    return true;
}

// -----------------------------------------------------------------------------

bool teo::MotorIpos::getTorqueErrorLimit(int j, double *limit) {
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( ! this->indexWithinRange(j) ) return false;

    CD_WARNING("Not implemented yet.\n");

    return true;
}

// -----------------------------------------------------------------------------

bool teo::MotorIpos::getTorqueErrorLimits(double *limits) {
    CD_INFO("\n");

    CD_WARNING("Not implemented yet.\n");

    return true;
}

// -----------------------------------------------------------------------------

bool teo::MotorIpos::resetTorquePid(int j) {
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( ! this->indexWithinRange(j) ) return false;

    CD_WARNING("Not implemented yet.\n");

    return true;
}

// -----------------------------------------------------------------------------

bool teo::MotorIpos::disableTorquePid(int j) {
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( ! this->indexWithinRange(j) ) return false;

    CD_WARNING("Not implemented yet.\n");

    return true;
}

// -----------------------------------------------------------------------------

bool teo::MotorIpos::enableTorquePid(int j) {
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( ! this->indexWithinRange(j) ) return false;

    CD_WARNING("Not implemented yet.\n");

    return true;
}

// -----------------------------------------------------------------------------

bool teo::MotorIpos::setTorqueOffset(int j, double v) {
    CD_INFO("(%d,%f)\n",j,v);

    //-- Check index within range
    if ( ! this->indexWithinRange(j) ) return false;

    CD_WARNING("Not implemented yet.\n");

    return true;
}

// -----------------------------------------------------------------------------
