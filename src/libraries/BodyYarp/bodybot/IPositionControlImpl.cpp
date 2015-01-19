// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "BodyBot.hpp"

// ------------------ IPositionControl Related ----------------------------------------

bool teo::BodyBot::getAxes(int *axes) {
    CD_INFO("\n");

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

bool teo::BodyBot::positionMove(int j, double ref) {
    CD_INFO("(%d,%f)\n",j,ref);

    //-- Check index within range
    if ( ! this->indexWithinRange(j) ) return false;

    return drivers[j]->positionMoveRaw( 0, ref );
}

// -----------------------------------------------------------------------------

bool teo::BodyBot::positionMove(const double *refs) {
    CD_INFO("\n");

    bool ok = true;
    for(int j=0; j<drivers.size(); j++)
    {
        ok &= this->positionMove(j,refs[j]);
    }
    return ok;
}

// -----------------------------------------------------------------------------

bool teo::BodyBot::relativeMove(int j, double delta) {
    CD_INFO("(%d,%f)\n",j,delta);

    //-- Check index within range
    if ( ! this->indexWithinRange(j) ) return false;

    return drivers[j]->relativeMoveRaw( 0, delta );
}

// -----------------------------------------------------------------------------

bool teo::BodyBot::relativeMove(const double *deltas) {
    CD_INFO("\n");

    bool ok = true;
    for(int j=0; j<drivers.size(); j++)
    {
        ok &= this->relativeMove(j,deltas[j]);
    }
    return ok;
}

// -----------------------------------------------------------------------------

bool teo::BodyBot::checkMotionDone(int j, bool *flag) {
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( ! this->indexWithinRange(j) ) return false;

    return drivers[j]->checkMotionDoneRaw( 0, flag );
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

    return drivers[j]->setRefSpeedRaw( 0, sp );
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

    return drivers[j]->setRefAccelerationRaw( 0, acc );
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

    return drivers[j]->getRefSpeedRaw( 0, ref);
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

    return drivers[j]->getRefAccelerationRaw( 0, acc );
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

    return drivers[j]->stopRaw (0);
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

