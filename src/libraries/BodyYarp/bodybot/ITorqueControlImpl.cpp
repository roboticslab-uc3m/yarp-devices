// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "BodyBot.hpp"

// ------------------- ITorqueControl Related ------------------------------------

bool teo::BodyBot::setTorqueMode() {
    CD_INFO("\n");

    bool ok = true;
    for(int j=0; j<drivers.size(); j++)
    {
        ok &= this->setTorqueMode(j);
    }
    return ok;
}

// -----------------------------------------------------------------------------

bool teo::BodyBot::getRefTorques(double *t){
    CD_INFO("\n");

    bool ok = true;
    for(int j=0; j<drivers.size(); j++)
    {
        ok &= this->getRefTorque(j, t);
    }
    return ok;
}

// -----------------------------------------------------------------------------

bool teo::BodyBot::getRefTorque(int j, double *t) {
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( ! this->indexWithinRange(j) ) return false;

    return drivers[j]->getRefTorqueRaw( 0, t );
}

// -----------------------------------------------------------------------------

bool teo::BodyBot::setRefTorques(const double *t) {
    CD_INFO("\n");

    bool ok = true;
    for(int j=0; j<drivers.size(); j++)
    {
        ok &= this->setRefTorque(0, t[j]);
    }
    return ok;
}


// -----------------------------------------------------------------------------

bool teo::BodyBot::setRefTorque(int j, double t) {
    CD_INFO("(%d,%f)\n",j,t);

    //-- Check index within range
    if ( ! this->indexWithinRange(j) ) return false;

    return drivers[j]->setRefTorqueRaw( 0, t );
}

// -----------------------------------------------------------------------------

bool teo::BodyBot::getBemfParam(int j, double *bemf) {
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( ! this->indexWithinRange(j) ) return false;

    return drivers[j]->getBemfParamRaw( 0, bemf );;
}

// -----------------------------------------------------------------------------

bool teo::BodyBot::setBemfParam(int j, double bemf) {
    CD_INFO("(%d,%f)\n",j,bemf);

    //-- Check index within range
    if ( ! this->indexWithinRange(j) ) return false;

    return drivers[j]->setBemfParamRaw( 0, bemf );;
}

// -----------------------------------------------------------------------------

bool teo::BodyBot::setTorquePid(int j, const Pid &pid) {
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( ! this->indexWithinRange(j) ) return false;

    return drivers[j]->setTorquePidRaw( 0, pid );;
}

// -----------------------------------------------------------------------------

bool teo::BodyBot::getTorque(int j, double *t) {
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( ! this->indexWithinRange(j) ) return false;

    return drivers[j]->getTorqueRaw( 0, t );;
}

// -----------------------------------------------------------------------------

bool teo::BodyBot::getTorques(double *t) {
    CD_INFO("\n");

    bool ok = true;
    for(int j=0; j<drivers.size(); j++)
    {
        ok &= this->getTorque(j, t);
    }
    return ok;
}

// -----------------------------------------------------------------------------

bool teo::BodyBot::getTorqueRange(int j, double *min, double *max) {
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( ! this->indexWithinRange(j) ) return false;

    return drivers[j]->getTorqueRangeRaw( 0, min, max );
}

// -----------------------------------------------------------------------------

bool teo::BodyBot::getTorqueRanges(double *min, double *max) {
    CD_INFO("\n");

    bool ok = true;
    for(int j=0; j<drivers.size(); j++)
    {
        ok &= this->getTorqueRange(j, min, max);
    }
    return ok;
}

// -----------------------------------------------------------------------------

bool teo::BodyBot::setTorquePids(const Pid *pids) {
    CD_INFO("\n");

    CD_WARNING("Not implemented yet.\n");

    return true;
}

// -----------------------------------------------------------------------------

bool teo::BodyBot::setTorqueErrorLimit(int j, double limit) {
    CD_INFO("(%d,%f)\n",j,limit);

    //-- Check index within range
    if ( ! this->indexWithinRange(j) ) return false;

    CD_WARNING("Not implemented yet.\n");

    return true;
}

// -----------------------------------------------------------------------------

bool teo::BodyBot::setTorqueErrorLimits(const double *limits) {
    CD_INFO("\n");

    CD_WARNING("Not implemented yet.\n");

    return true;
}

// -----------------------------------------------------------------------------

bool teo::BodyBot::getTorqueError(int j, double *err) {
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( ! this->indexWithinRange(j) ) return false;

    CD_WARNING("Not implemented yet.\n");

    return true;
}

// -----------------------------------------------------------------------------

bool teo::BodyBot::getTorqueErrors(double *errs) {
    CD_INFO("\n");

    CD_WARNING("Not implemented yet.\n");

    return true;
}

// -----------------------------------------------------------------------------

bool teo::BodyBot::getTorquePidOutput(int j, double *out) {
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( ! this->indexWithinRange(j) ) return false;

    CD_WARNING("Not implemented yet.\n");

    return true;
}

// -----------------------------------------------------------------------------

bool teo::BodyBot::getTorquePidOutputs(double *outs) {
    CD_INFO("\n");

    CD_WARNING("Not implemented yet.\n");

    return true;
}

// -----------------------------------------------------------------------------

bool teo::BodyBot::getTorquePid(int j, Pid *pid) {
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( ! this->indexWithinRange(j) ) return false;

    CD_WARNING("Not implemented yet.\n");

    return true;
}

// -----------------------------------------------------------------------------

bool teo::BodyBot::getTorquePids(Pid *pids){
    CD_INFO("\n");

    CD_WARNING("Not implemented yet.\n");

    return true;
}

// -----------------------------------------------------------------------------

bool teo::BodyBot::getTorqueErrorLimit(int j, double *limit) {
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( ! this->indexWithinRange(j) ) return false;

    CD_WARNING("Not implemented yet.\n");

    return true;
}

// -----------------------------------------------------------------------------

bool teo::BodyBot::getTorqueErrorLimits(double *limits) {
    CD_INFO("\n");

    CD_WARNING("Not implemented yet.\n");

    return true;
}

// -----------------------------------------------------------------------------

bool teo::BodyBot::resetTorquePid(int j) {
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( ! this->indexWithinRange(j) ) return false;

    CD_WARNING("Not implemented yet.\n");

    return true;
}

// -----------------------------------------------------------------------------

bool teo::BodyBot::disableTorquePid(int j) {
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( ! this->indexWithinRange(j) ) return false;

    CD_WARNING("Not implemented yet.\n");

    return true;
}

// -----------------------------------------------------------------------------

bool teo::BodyBot::enableTorquePid(int j) {
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( ! this->indexWithinRange(j) ) return false;

    CD_WARNING("Not implemented yet.\n");

    return true;
}

// -----------------------------------------------------------------------------

bool teo::BodyBot::setTorqueOffset(int j, double v) {
    CD_INFO("(%d,%f)\n",j,v);

    //-- Check index within range
    if ( ! this->indexWithinRange(j) ) return false;

    CD_WARNING("Not implemented yet.\n");

    return true;
}

// -----------------------------------------------------------------------------
