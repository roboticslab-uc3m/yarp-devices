// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusControlboard.hpp"

// ------------------ IPositionControl Related ----------------------------------------

bool teo::CanBusControlboard::getAxes(int *axes) {
    CD_INFO("\n");

    *axes = drivers.size();

    return true;
}

// -----------------------------------------------------------------------------

bool teo::CanBusControlboard::setPositionMode() {
    CD_INFO("\n");

    bool ok = true;
    for(int j=0; j<drivers.size(); j++)
    {
        ok &= this->setPositionMode(j);
    }
    return ok;
}

// -----------------------------------------------------------------------------

bool teo::CanBusControlboard::positionMove(int j, double ref) {
    CD_INFO("(%d,%f)\n",j,ref);

    //-- Check index within range
    if ( ! this->indexWithinRange(j) ) return false;

    return iPositionControlRaw[j]->positionMoveRaw( 0, ref );
}

// -----------------------------------------------------------------------------

bool teo::CanBusControlboard::positionMove(const double *refs) {
    CD_INFO("\n");

    bool ok = true;
    for(int j=0; j<drivers.size(); j++)
    {
        ok &= this->positionMove(j,refs[j]);
    }
    return ok;
}

// -----------------------------------------------------------------------------

bool teo::CanBusControlboard::relativeMove(int j, double delta) {
    CD_INFO("(%d,%f)\n",j,delta);

    //-- Check index within range
    if ( ! this->indexWithinRange(j) ) return false;

    return iPositionControlRaw[j]->relativeMoveRaw( 0, delta );
}

// -----------------------------------------------------------------------------

bool teo::CanBusControlboard::relativeMove(const double *deltas) {
    CD_INFO("\n");

    bool ok = true;
    for(int j=0; j<drivers.size(); j++)
    {
        ok &= this->relativeMove(j,deltas[j]);
    }
    return ok;
}

// -----------------------------------------------------------------------------

bool teo::CanBusControlboard::checkMotionDone(int j, bool *flag) {
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( ! this->indexWithinRange(j) ) return false;

    return iPositionControlRaw[j]->checkMotionDoneRaw( 0, flag );
}

// -----------------------------------------------------------------------------

bool teo::CanBusControlboard::checkMotionDone(bool *flag) {
    CD_INFO("\n");
    *flag = true;
    bool ok = true;
    for(int j=0; j<drivers.size(); j++)
    {
        bool tmpFlag;
        ok &= this->checkMotionDone(j,&tmpFlag);
        *flag &= tmpFlag;
    }
    return ok;
}

// -----------------------------------------------------------------------------

bool teo::CanBusControlboard::setRefSpeed(int j, double sp) {
    CD_INFO("(%d, %f)\n",j,sp);

    //-- Check index within range
    if ( ! this->indexWithinRange(j) ) return false;

    return iPositionControlRaw[j]->setRefSpeedRaw( 0, sp );
}

// -----------------------------------------------------------------------------

bool teo::CanBusControlboard::setRefSpeeds(const double *spds) {
    CD_INFO("\n");

    bool ok = true;
    for(unsigned int i=0;i<drivers.size();i++)
        ok &= setRefSpeed(i,spds[i]);
    return ok;
}

// -----------------------------------------------------------------------------

bool teo::CanBusControlboard::setRefAcceleration(int j, double acc) {
    CD_INFO("(%d, %f)\n",j,acc);

    //-- Check index within range
    if ( ! this->indexWithinRange(j) ) return false;

    return iPositionControlRaw[j]->setRefAccelerationRaw( 0, acc );
}

// -----------------------------------------------------------------------------

bool teo::CanBusControlboard::setRefAccelerations(const double *accs) {
    CD_INFO("\n");

    bool ok = true;
    for(unsigned int i=0;i<drivers.size();i++)
        ok &= setRefAcceleration(i,accs[i]);
    return ok;
}

// -----------------------------------------------------------------------------

bool teo::CanBusControlboard::getRefSpeed(int j, double *ref) {
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( ! this->indexWithinRange(j) ) return false;

    return iPositionControlRaw[j]->getRefSpeedRaw( 0, ref);
}

// -----------------------------------------------------------------------------

bool teo::CanBusControlboard::getRefSpeeds(double *spds) {
    CD_INFO("\n");

    bool ok = true;
    for(unsigned int i=0;i<drivers.size();i++)
        ok &= getRefSpeed(i,&spds[i]);
    return ok;
}

// -----------------------------------------------------------------------------

bool teo::CanBusControlboard::getRefAcceleration(int j, double *acc) {
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( ! this->indexWithinRange(j) ) return false;

    return iPositionControlRaw[j]->getRefAccelerationRaw( 0, acc );
}

// -----------------------------------------------------------------------------

bool teo::CanBusControlboard::getRefAccelerations(double *accs) {
    CD_INFO("\n");

    bool ok = true;
    for(unsigned int i=0;i<drivers.size();i++)
        ok &= getRefAcceleration(i,&accs[i]);
    return ok;
}

// -----------------------------------------------------------------------------

bool teo::CanBusControlboard::stop(int j) {
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( ! this->indexWithinRange(j) ) return false;

    return iPositionControlRaw[j]->stopRaw (0);
}

// -----------------------------------------------------------------------------

bool teo::CanBusControlboard::stop() {
    CD_INFO("\n");

    bool ok = true;
    for(unsigned int i=0;i<drivers.size();i++)
        ok &= stop(i);
    return ok;
}

// -----------------------------------------------------------------------------

