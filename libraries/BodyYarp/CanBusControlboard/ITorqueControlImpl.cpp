// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusControlboard.hpp"

// ------------------- ITorqueControl Related ------------------------------------

bool teo::CanBusControlboard::setTorqueMode() {
    CD_INFO("\n");

    bool ok = true;
    for(int j=0; j<nodes.size(); j++)
    {
        ok &= this->setTorqueMode(j);
    }
    return ok;
}

// -----------------------------------------------------------------------------

bool teo::CanBusControlboard::getRefTorques(double *t){
    CD_INFO("\n");

    bool ok = true;
    for(int j=0; j<nodes.size(); j++)
    {
        ok &= this->getRefTorque(j, &(t[j]));
    }
    return ok;
}

// -----------------------------------------------------------------------------

bool teo::CanBusControlboard::getRefTorque(int j, double *t) {
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( ! this->indexWithinRange(j) ) return false;

    return iTorqueControlRaw[j]->getRefTorqueRaw( 0, t );
}

// -----------------------------------------------------------------------------

bool teo::CanBusControlboard::setRefTorques(const double *t) {
    CD_INFO("\n");

    bool ok = true;
    for(int j=0; j<nodes.size(); j++)
    {
        ok &= this->setRefTorque(j, t[j]);
    }
    return ok;
}


// -----------------------------------------------------------------------------

bool teo::CanBusControlboard::setRefTorque(int j, double t) {
    CD_INFO("(%d,%f)\n",j,t);

    //-- Check index within range
    if ( ! this->indexWithinRange(j) ) return false;

    return iTorqueControlRaw[j]->setRefTorqueRaw( 0, t );
}

// -----------------------------------------------------------------------------

bool teo::CanBusControlboard::getBemfParam(int j, double *bemf) {
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( ! this->indexWithinRange(j) ) return false;

    return iTorqueControlRaw[j]->getBemfParamRaw( 0, bemf );;
}

// -----------------------------------------------------------------------------

bool teo::CanBusControlboard::setBemfParam(int j, double bemf) {
    CD_INFO("(%d,%f)\n",j,bemf);

    //-- Check index within range
    if ( ! this->indexWithinRange(j) ) return false;

    return iTorqueControlRaw[j]->setBemfParamRaw( 0, bemf );;
}

// -----------------------------------------------------------------------------

bool teo::CanBusControlboard::setTorquePid(int j, const Pid &pid) {
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( ! this->indexWithinRange(j) ) return false;

    return iTorqueControlRaw[j]->setTorquePidRaw( 0, pid );;
}

// -----------------------------------------------------------------------------

bool teo::CanBusControlboard::getTorque(int j, double *t) {
    //CD_INFO("(%d)\n",j);  //-- Too verbose in controlboardwrapper2 stream.

    //-- Check index within range
    if ( ! this->indexWithinRange(j) ) return false;

    return iTorqueControlRaw[j]->getTorqueRaw( 0, t );;
}

// -----------------------------------------------------------------------------

bool teo::CanBusControlboard::getTorques(double *t) {
    CD_INFO("\n");

    bool ok = true;
    for(int j=0; j<nodes.size(); j++)
    {
        ok &= this->getTorque(j, &(t[j]));
    }
    return ok;
}

// -----------------------------------------------------------------------------

bool teo::CanBusControlboard::getTorqueRange(int j, double *min, double *max) {
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( ! this->indexWithinRange(j) ) return false;

    return iTorqueControlRaw[j]->getTorqueRangeRaw( 0, min, max );
}

// -----------------------------------------------------------------------------

bool teo::CanBusControlboard::getTorqueRanges(double *min, double *max) {
    CD_INFO("\n");

    bool ok = true;
    for(int j=0; j<nodes.size(); j++)
    {
        ok &= this->getTorqueRange(j, min, max);
    }
    return ok;
}

// -----------------------------------------------------------------------------

bool teo::CanBusControlboard::setTorquePids(const Pid *pids) {
    CD_INFO("\n");

    bool ok = true;
    for(int j=0; j<nodes.size(); j++)
    {
        ok &= this->setTorquePid(j, pids[j]);
    }
    return ok;
}

// -----------------------------------------------------------------------------

bool teo::CanBusControlboard::setTorqueErrorLimit(int j, double limit) {
    CD_INFO("(%d,%f)\n",j,limit);

    //-- Check index within range
    if ( ! this->indexWithinRange(j) ) return false;

    return iTorqueControlRaw[j]->setTorqueErrorLimitRaw( 0, limit );
}

// -----------------------------------------------------------------------------

bool teo::CanBusControlboard::setTorqueErrorLimits(const double *limits) {
    CD_INFO("\n");

    bool ok = true;
    for(int j=0; j<nodes.size(); j++)
    {
        ok &= this->setTorqueErrorLimit(j, limits[j]);
    }
    return ok;
}

// -----------------------------------------------------------------------------

bool teo::CanBusControlboard::getTorqueError(int j, double *err) {
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( ! this->indexWithinRange(j) ) return false;

    return iTorqueControlRaw[j]->getTorqueErrorRaw( 0, err );
}

// -----------------------------------------------------------------------------

bool teo::CanBusControlboard::getTorqueErrors(double *errs) {
    CD_INFO("\n");

    bool ok = true;
    for(int j=0; j<nodes.size(); j++)
    {
        ok &= this->getTorqueError(j, &(errs[j]));
    }
    return ok;
}

// -----------------------------------------------------------------------------

bool teo::CanBusControlboard::getTorquePidOutput(int j, double *out) {
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( ! this->indexWithinRange(j) ) return false;

    return iTorqueControlRaw[j]->getTorquePidOutputRaw( 0, out );
}

// -----------------------------------------------------------------------------

bool teo::CanBusControlboard::getTorquePidOutputs(double *outs) {
    CD_INFO("\n");

    bool ok = true;
    for(int j=0; j<nodes.size(); j++)
    {
        ok &= this->getTorquePidOutput(j, &(outs[j]));
    }
    return ok;
}

// -----------------------------------------------------------------------------

bool teo::CanBusControlboard::getTorquePid(int j, Pid *pid) {
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( ! this->indexWithinRange(j) ) return false;

    return iTorqueControlRaw[j]->getTorquePidRaw( 0, pid );
}

// -----------------------------------------------------------------------------

bool teo::CanBusControlboard::getTorquePids(Pid *pids){
    CD_INFO("\n");

    bool ok = true;
    for(int j=0; j<nodes.size(); j++)
    {
        ok &= this->getTorquePid(j, &(pids[j]));
    }
    return ok;
}

// -----------------------------------------------------------------------------

bool teo::CanBusControlboard::getTorqueErrorLimit(int j, double *limit) {
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( ! this->indexWithinRange(j) ) return false;

    return iTorqueControlRaw[j]->getTorqueErrorLimitRaw( 0, limit );
}

// -----------------------------------------------------------------------------

bool teo::CanBusControlboard::getTorqueErrorLimits(double *limits) {
    CD_INFO("\n");

    bool ok = true;
    for(int j=0; j<nodes.size(); j++)
    {
        ok &= this->getTorqueErrorLimit(j, &(limits[j]));
    }
    return ok;
}

// -----------------------------------------------------------------------------

bool teo::CanBusControlboard::resetTorquePid(int j) {
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( ! this->indexWithinRange(j) ) return false;

    return iTorqueControlRaw[j]->resetTorquePidRaw(0);
}

// -----------------------------------------------------------------------------

bool teo::CanBusControlboard::disableTorquePid(int j) {
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( ! this->indexWithinRange(j) ) return false;

    return iTorqueControlRaw[j]->disableTorquePidRaw(0);
}

// -----------------------------------------------------------------------------

bool teo::CanBusControlboard::enableTorquePid(int j) {
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( ! this->indexWithinRange(j) ) return false;

    return iTorqueControlRaw[j]->enableTorquePidRaw(0);
}

// -----------------------------------------------------------------------------

bool teo::CanBusControlboard::setTorqueOffset(int j, double v) {
    CD_INFO("(%d,%f)\n",j,v);

    //-- Check index within range
    if ( ! this->indexWithinRange(j) ) return false;

    return iTorqueControlRaw[j]->setTorqueOffsetRaw( 0, v );
}

// -----------------------------------------------------------------------------
