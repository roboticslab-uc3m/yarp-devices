// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusControlboard.hpp"

// ------------------ IPositionControl Related ----------------------------------------

bool roboticslab::CanBusControlboard::getAxes(int *axes)
{
    CD_DEBUG("\n");

    *axes = nodes.size();

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::CanBusControlboard::positionMove(int j, double ref)
{
    CD_DEBUG("(%d, %f)\n",j,ref);

    //-- Check index within range
    if ( ! this->indexWithinRange(j) ) return false;

    return iPositionControl2Raw[j]->positionMoveRaw( 0, ref );
}

// -----------------------------------------------------------------------------

bool roboticslab::CanBusControlboard::positionMove(const double *refs)
{
    CD_DEBUG("\n");

    bool ok = true;
    for(int j=0; j<nodes.size(); j++)
    {
        ok &= this->positionMove(j,refs[j]);
    }
    return ok;
}

// -----------------------------------------------------------------------------

bool roboticslab::CanBusControlboard::relativeMove(int j, double delta)
{
    CD_DEBUG("(%d, %f)\n",j,delta);

    //-- Check index within range
    if ( ! this->indexWithinRange(j) ) return false;

    return iPositionControl2Raw[j]->relativeMoveRaw( 0, delta );
}

// -----------------------------------------------------------------------------

bool roboticslab::CanBusControlboard::relativeMove(const double *deltas)
{
    CD_DEBUG("\n");

    bool ok = true;
    for(int j=0; j<nodes.size(); j++)
    {
        ok &= this->relativeMove(j,deltas[j]);
    }
    return ok;
}

// -----------------------------------------------------------------------------

bool roboticslab::CanBusControlboard::checkMotionDone(int j, bool *flag)
{
    CD_DEBUG("(%d)\n",j);

    //-- Check index within range
    if ( ! this->indexWithinRange(j) ) return false;

    return iPositionControl2Raw[j]->checkMotionDoneRaw( 0, flag );
}

// -----------------------------------------------------------------------------

bool roboticslab::CanBusControlboard::checkMotionDone(bool *flag)
{
    CD_DEBUG("\n");
    *flag = true;
    bool ok = true;
    for(int j=0; j<nodes.size(); j++)
    {
        bool tmpFlag;
        ok &= this->checkMotionDone(j,&tmpFlag);
        *flag &= tmpFlag;
    }
    return ok;
}

// -----------------------------------------------------------------------------

bool roboticslab::CanBusControlboard::setRefSpeed(int j, double sp)
{
    CD_DEBUG("(%d, %f)\n",j,sp);

    //-- Check index within range
    if ( ! this->indexWithinRange(j) ) return false;

    return iPositionControl2Raw[j]->setRefSpeedRaw( 0, sp );
}

// -----------------------------------------------------------------------------

bool roboticslab::CanBusControlboard::setRefSpeeds(const double *spds)
{
    CD_DEBUG("\n");

    bool ok = true;
    for(unsigned int i=0; i<nodes.size(); i++)
        ok &= setRefSpeed(i,spds[i]);
    return ok;
}

// -----------------------------------------------------------------------------

bool roboticslab::CanBusControlboard::setRefAcceleration(int j, double acc)
{
    CD_DEBUG("(%d, %f)\n",j,acc);

    //-- Check index within range
    if ( ! this->indexWithinRange(j) ) return false;

    return iPositionControl2Raw[j]->setRefAccelerationRaw( 0, acc );
}

// -----------------------------------------------------------------------------

bool roboticslab::CanBusControlboard::setRefAccelerations(const double *accs)
{
    CD_DEBUG("\n");

    bool ok = true;
    for(unsigned int i=0; i<nodes.size(); i++)
        ok &= setRefAcceleration(i,accs[i]);
    return ok;
}

// -----------------------------------------------------------------------------

bool roboticslab::CanBusControlboard::getRefSpeed(int j, double *ref)
{
    CD_DEBUG("(%d)\n",j);

    //-- Check index within range
    if ( ! this->indexWithinRange(j) ) return false;

    return iPositionControl2Raw[j]->getRefSpeedRaw( 0, ref);
}

// -----------------------------------------------------------------------------

bool roboticslab::CanBusControlboard::getRefSpeeds(double *spds)
{
    CD_DEBUG("\n");

    bool ok = true;
    for(unsigned int i=0; i<nodes.size(); i++)
        ok &= getRefSpeed(i,&spds[i]);
    return ok;
}

// -----------------------------------------------------------------------------

bool roboticslab::CanBusControlboard::getRefAcceleration(int j, double *acc)
{
    CD_DEBUG("(%d)\n",j);

    //-- Check index within range
    if ( ! this->indexWithinRange(j) ) return false;

    return iPositionControl2Raw[j]->getRefAccelerationRaw( 0, acc );
}

// -----------------------------------------------------------------------------

bool roboticslab::CanBusControlboard::getRefAccelerations(double *accs)
{
    CD_DEBUG("\n");

    bool ok = true;
    for(unsigned int i=0; i<nodes.size(); i++)
        ok &= getRefAcceleration(i,&accs[i]);
    return ok;
}

// -----------------------------------------------------------------------------

bool roboticslab::CanBusControlboard::stop(int j)
{
    CD_DEBUG("(%d)\n",j);

    //-- Check index within range
    if ( ! this->indexWithinRange(j) ) return false;

    return iPositionControl2Raw[j]->stopRaw (0);
}

// -----------------------------------------------------------------------------

bool roboticslab::CanBusControlboard::stop()
{
    CD_DEBUG("\n");

    bool ok = true;
    for(unsigned int i=0; i<nodes.size(); i++)
        ok &= stop(i);
    return ok;
}

// ---------------------------- IPositionControl2 Related ---------------------

bool roboticslab::CanBusControlboard::positionMove(const int n_joint, const int *joints, const double *refs)
{
    CD_DEBUG("\n");

    bool ok = true;
    for(int j=0; j<n_joint; j++)
    {
        ok &= this->positionMove(joints[j],refs[j]);
    }
    return ok;
}

// -----------------------------------------------------------------------------

bool roboticslab::CanBusControlboard::relativeMove(const int n_joint, const int *joints, const double *deltas)
{
    CD_DEBUG("\n");

    bool ok = true;
    for(int j=0; j<n_joint; j++) // j<nodes.size()
    {
        ok &= this->relativeMove(joints[j],deltas[j]);
    }
    return ok;
}

// -----------------------------------------------------------------------------

bool roboticslab::CanBusControlboard::checkMotionDone(const int n_joint, const int *joints, bool *flags)
{
    CD_DEBUG("\n");
    *flags = true;
    bool ok = true;
    for(int j=0; j<n_joint; j++)
    {
        bool tmpFlag;
        ok &= this->checkMotionDone(joints[j],&tmpFlag);
        *flags &= tmpFlag;
    }
    return ok;
}

// -----------------------------------------------------------------------------

bool roboticslab::CanBusControlboard::setRefSpeeds(const int n_joint, const int *joints, const double *spds)
{
    CD_DEBUG("\n");

    bool ok = true;
    for(unsigned int i=0; i<n_joint; i++)
    {
        ok &= setRefSpeed(joints[i],spds[i]);
    }
    return ok;
}

// -----------------------------------------------------------------------------

bool roboticslab::CanBusControlboard::setRefAccelerations(const int n_joint, const int *joints, const double *accs)
{
    CD_DEBUG("\n");

    bool ok = true;
    for(unsigned int i=0; i<n_joint; i++)
    {
        ok &= setRefAcceleration(joints[i],accs[i]);
    }
    return ok;
}

// -----------------------------------------------------------------------------

bool roboticslab::CanBusControlboard::getRefSpeeds(const int n_joint, const int *joints, double *spds)
{
    CD_DEBUG("\n");

    bool ok = true;
    for(unsigned int i=0; i<n_joint; i++)
    {
        ok &= getRefSpeed(joints[i],&spds[i]);
    }
    return ok;
}

// -----------------------------------------------------------------------------

bool roboticslab::CanBusControlboard::getRefAccelerations(const int n_joint, const int *joints, double *accs)
{
    CD_DEBUG("\n");

    bool ok = true;
    for(unsigned int i=0; i<n_joint; i++)
    {
        ok &= getRefAcceleration(joints[i],&accs[i]);
    }
    return ok;
}

// -----------------------------------------------------------------------------

bool roboticslab::CanBusControlboard::stop(const int n_joint, const int *joints)
{
    CD_DEBUG("\n");

    bool ok = true;
    for(unsigned int i=0; i<n_joint; i++)
    {
            ok &= stop(i);
    }
    return ok;
}

// -----------------------------------------------------------------------------

bool roboticslab::CanBusControlboard::getTargetPosition(const int joint, double *ref)
{
    CD_DEBUG("\n");

    return iPositionControl2Raw[joint]->getTargetPositionRaw(0, ref);

}

// -----------------------------------------------------------------------------

bool roboticslab::CanBusControlboard::getTargetPositions(double *refs)
{
    CD_DEBUG("\n");

    bool ok = true;
    for(unsigned int i=0; i<nodes.size(); i++)
    {
        ok &= getTargetPosition(i,&(refs[i]));
    }
    return ok;
}

// -----------------------------------------------------------------------------

bool roboticslab::CanBusControlboard::getTargetPositions(const int n_joint, const int *joints, double *refs)
{
    CD_DEBUG("\n");

    bool ok = true;
    for(unsigned int i=0; i<n_joint; i++)
    {
        ok &= getTargetPosition(joints[i],&(refs[i]));
    }
    return ok;
}
