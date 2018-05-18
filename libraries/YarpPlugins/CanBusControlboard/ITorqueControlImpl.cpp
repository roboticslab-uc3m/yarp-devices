// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusControlboard.hpp"

// ------------------- ITorqueControl Related ------------------------------------

bool roboticslab::CanBusControlboard::getRefTorques(double *t)
{
    CD_DEBUG("\n");

    bool ok = true;
    for(int j=0; j<nodes.size(); j++)
    {
        ok &= this->getRefTorque(j, &(t[j]));
    }
    return ok;
}

// -----------------------------------------------------------------------------

bool roboticslab::CanBusControlboard::getRefTorque(int j, double *t)
{
    CD_DEBUG("(%d)\n",j);

    //-- Check index within range
    if ( ! this->indexWithinRange(j) ) return false;

    return iTorqueControlRaw[j]->getRefTorqueRaw( 0, t );
}

// -----------------------------------------------------------------------------

bool roboticslab::CanBusControlboard::setRefTorques(const double *t)
{
    CD_DEBUG("\n");

    bool ok = true;
    for(int j=0; j<nodes.size(); j++)
    {
        ok &= this->setRefTorque(j, t[j]);
    }
    return ok;
}


// -----------------------------------------------------------------------------

bool roboticslab::CanBusControlboard::setRefTorque(int j, double t)
{
    CD_DEBUG("(%d,%f)\n",j,t);

    //-- Check index within range
    if ( ! this->indexWithinRange(j) ) return false;

    return iTorqueControlRaw[j]->setRefTorqueRaw( 0, t );
}

// -----------------------------------------------------------------------------

bool roboticslab::CanBusControlboard::setRefTorques(const int n_joint, const int *joints, const double *t)
{
    CD_DEBUG("(%d)\n",n_joint);

    bool ok = true;
    for(int j=0; j<n_joint; j++)
    {
        ok &= this->setRefTorque(joints[j],t[j]);
    }
    return ok;
}

// -----------------------------------------------------------------------------

bool roboticslab::CanBusControlboard::getMotorTorqueParams(int j,  yarp::dev::MotorTorqueParameters *params)
{
    CD_DEBUG("(%d)\n",j);

    //-- Check index within range
    if ( ! this->indexWithinRange(j) ) return false;

    return iTorqueControlRaw[j]->getMotorTorqueParamsRaw( 0, params );
}

// -----------------------------------------------------------------------------

bool roboticslab::CanBusControlboard::setMotorTorqueParams(int j, const yarp::dev::MotorTorqueParameters params)
{
    CD_DEBUG("(%d)\n",j);

    //-- Check index within range
    if ( ! this->indexWithinRange(j) ) return false;

    return iTorqueControlRaw[j]->setMotorTorqueParamsRaw( 0, params );
}

// -----------------------------------------------------------------------------

bool roboticslab::CanBusControlboard::getTorque(int j, double *t)
{
    //CD_INFO("(%d)\n",j);  //-- Too verbose in controlboardwrapper2 stream.

    //-- Check index within range
    if ( ! this->indexWithinRange(j) ) return false;

    return iTorqueControlRaw[j]->getTorqueRaw( 0, t );;
}

// -----------------------------------------------------------------------------

bool roboticslab::CanBusControlboard::getTorques(double *t)
{
    CD_DEBUG("\n");

    bool ok = true;
    for(int j=0; j<nodes.size(); j++)
    {
        ok &= this->getTorque(j, &(t[j]));
    }
    return ok;
}

// -----------------------------------------------------------------------------

bool roboticslab::CanBusControlboard::getTorqueRange(int j, double *min, double *max)
{
    CD_DEBUG("(%d)\n",j);

    //-- Check index within range
    if ( ! this->indexWithinRange(j) ) return false;

    return iTorqueControlRaw[j]->getTorqueRangeRaw( 0, min, max );
}

// -----------------------------------------------------------------------------

bool roboticslab::CanBusControlboard::getTorqueRanges(double *min, double *max)
{
    CD_DEBUG("\n");

    bool ok = true;
    for(int j=0; j<nodes.size(); j++)
    {
        ok &= this->getTorqueRange(j, min, max);
    }
    return ok;
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_MAJOR != 3
bool roboticslab::CanBusControlboard::getBemfParam(int j, double *bemf)
{
    CD_DEBUG("(%d)\n",j);

    //-- Check index within range
    if ( ! this->indexWithinRange(j) ) return false;

    yarp::dev::MotorTorqueParameters params;

    if (!getMotorTorqueParams(j, &params))
    {
        return false;
    }

    *bemf = params.bemf;

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::CanBusControlboard::setBemfParam(int j, double bemf)
{
    CD_DEBUG("(%d,%f)\n",j,bemf);

    //-- Check index within range
    if ( ! this->indexWithinRange(j) ) return false;

    yarp::dev::MotorTorqueParameters params;

    if (!getMotorTorqueParams(j, &params))
    {
        return false;
    }

    params.bemf = bemf;

    return setMotorTorqueParams(j, params);
}

// -----------------------------------------------------------------------------
#endif // YARP_VERSION_MAJOR != 3
