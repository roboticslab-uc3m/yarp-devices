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

    return iTorqueControlRaw[j]->getBemfParamRaw( 0, bemf );;
}

// -----------------------------------------------------------------------------

bool roboticslab::CanBusControlboard::setBemfParam(int j, double bemf)
{
    CD_DEBUG("(%d,%f)\n",j,bemf);

    //-- Check index within range
    if ( ! this->indexWithinRange(j) ) return false;

    return iTorqueControlRaw[j]->setBemfParamRaw( 0, bemf );;
}

// -----------------------------------------------------------------------------

bool roboticslab::CanBusControlboard::setTorquePid(int j, const yarp::dev::Pid &pid)
{
    CD_DEBUG("(%d)\n",j);

    //-- Check index within range
    if ( ! this->indexWithinRange(j) ) return false;

    return iTorqueControlRaw[j]->setTorquePidRaw( 0, pid );;
}

// -----------------------------------------------------------------------------

bool roboticslab::CanBusControlboard::setTorquePids(const yarp::dev::Pid *pids)
{
    CD_DEBUG("\n");

    bool ok = true;
    for(int j=0; j<nodes.size(); j++)
    {
        ok &= this->setTorquePid(j, pids[j]);
    }
    return ok;
}

// -----------------------------------------------------------------------------

bool roboticslab::CanBusControlboard::setTorqueErrorLimit(int j, double limit)
{
    CD_DEBUG("(%d,%f)\n",j,limit);

    //-- Check index within range
    if ( ! this->indexWithinRange(j) ) return false;

    return iTorqueControlRaw[j]->setTorqueErrorLimitRaw( 0, limit );
}

// -----------------------------------------------------------------------------

bool roboticslab::CanBusControlboard::setTorqueErrorLimits(const double *limits)
{
    CD_DEBUG("\n");

    bool ok = true;
    for(int j=0; j<nodes.size(); j++)
    {
        ok &= this->setTorqueErrorLimit(j, limits[j]);
    }
    return ok;
}

// -----------------------------------------------------------------------------

bool roboticslab::CanBusControlboard::getTorqueError(int j, double *err)
{
    CD_DEBUG("(%d)\n",j);

    //-- Check index within range
    if ( ! this->indexWithinRange(j) ) return false;

    return iTorqueControlRaw[j]->getTorqueErrorRaw( 0, err );
}

// -----------------------------------------------------------------------------

bool roboticslab::CanBusControlboard::getTorqueErrors(double *errs)
{
    CD_DEBUG("\n");

    bool ok = true;
    for(int j=0; j<nodes.size(); j++)
    {
        ok &= this->getTorqueError(j, &(errs[j]));
    }
    return ok;
}

// -----------------------------------------------------------------------------

bool roboticslab::CanBusControlboard::getTorquePidOutput(int j, double *out)
{
    CD_DEBUG("(%d)\n",j);

    //-- Check index within range
    if ( ! this->indexWithinRange(j) ) return false;

    return iTorqueControlRaw[j]->getTorquePidOutputRaw( 0, out );
}

// -----------------------------------------------------------------------------

bool roboticslab::CanBusControlboard::getTorquePidOutputs(double *outs)
{
    CD_DEBUG("\n");

    bool ok = true;
    for(int j=0; j<nodes.size(); j++)
    {
        ok &= this->getTorquePidOutput(j, &(outs[j]));
    }
    return ok;
}

// -----------------------------------------------------------------------------

bool roboticslab::CanBusControlboard::getTorquePid(int j, yarp::dev::Pid *pid)
{
    CD_DEBUG("(%d)\n",j);

    //-- Check index within range
    if ( ! this->indexWithinRange(j) ) return false;

    return iTorqueControlRaw[j]->getTorquePidRaw( 0, pid );
}

// -----------------------------------------------------------------------------

bool roboticslab::CanBusControlboard::getTorquePids(yarp::dev::Pid *pids)
{
    CD_DEBUG("\n");

    bool ok = true;
    for(int j=0; j<nodes.size(); j++)
    {
        ok &= this->getTorquePid(j, &(pids[j]));
    }
    return ok;
}

// -----------------------------------------------------------------------------

bool roboticslab::CanBusControlboard::getTorqueErrorLimit(int j, double *limit)
{
    CD_DEBUG("(%d)\n",j);

    //-- Check index within range
    if ( ! this->indexWithinRange(j) ) return false;

    return iTorqueControlRaw[j]->getTorqueErrorLimitRaw( 0, limit );
}

// -----------------------------------------------------------------------------

bool roboticslab::CanBusControlboard::getTorqueErrorLimits(double *limits)
{
    CD_DEBUG("\n");

    bool ok = true;
    for(int j=0; j<nodes.size(); j++)
    {
        ok &= this->getTorqueErrorLimit(j, &(limits[j]));
    }
    return ok;
}

// -----------------------------------------------------------------------------

bool roboticslab::CanBusControlboard::resetTorquePid(int j)
{
    CD_DEBUG("(%d)\n",j);

    //-- Check index within range
    if ( ! this->indexWithinRange(j) ) return false;

    return iTorqueControlRaw[j]->resetTorquePidRaw(0);
}

// -----------------------------------------------------------------------------

bool roboticslab::CanBusControlboard::disableTorquePid(int j)
{
    CD_DEBUG("(%d)\n",j);

    //-- Check index within range
    if ( ! this->indexWithinRange(j) ) return false;

    return iTorqueControlRaw[j]->disableTorquePidRaw(0);
}

// -----------------------------------------------------------------------------

bool roboticslab::CanBusControlboard::enableTorquePid(int j)
{
    CD_DEBUG("(%d)\n",j);

    //-- Check index within range
    if ( ! this->indexWithinRange(j) ) return false;

    return iTorqueControlRaw[j]->enableTorquePidRaw(0);
}

// -----------------------------------------------------------------------------

bool roboticslab::CanBusControlboard::setTorqueOffset(int j, double v)
{
    CD_DEBUG("(%d,%f)\n",j,v);

    //-- Check index within range
    if ( ! this->indexWithinRange(j) ) return false;

    return iTorqueControlRaw[j]->setTorqueOffsetRaw( 0, v );
}

// -----------------------------------------------------------------------------
#endif // YARP_VERSION_MAJOR != 3
