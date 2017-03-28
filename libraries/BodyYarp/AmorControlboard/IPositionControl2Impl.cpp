// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "AmorControlboard.hpp"

// ------------------- IPositionControl2 Related --------------------------------

bool roboticslab::AmorControlboard::positionMove(const int n_joint, const int *joints, const double *refs)
{
    CD_DEBUG("(%d)\n", n_joint);
    if (!indexWithinRange(n_joint))
        return false;
    // must implement mask!
    return positionMove(refs);
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::relativeMove(const int n_joint, const int *joints, const double *deltas)
{
    CD_DEBUG("(%d)\n", n_joint);
    if (!indexWithinRange(n_joint))
        return false;
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::checkMotionDone(const int n_joint, const int *joints, bool *flags)
{
    CD_DEBUG("(%d)\n", n_joint);
    if (!indexWithinRange(n_joint))
        return false;
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::setRefSpeeds(const int n_joint, const int *joints, const double *spds)
{
    CD_DEBUG("(%d)\n", n_joint);
    if (!indexWithinRange(n_joint))
        return false;
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::setRefAccelerations(const int n_joint, const int *joints, const double *accs)
{
    CD_DEBUG("(%d)\n", n_joint);
    if (!indexWithinRange(n_joint))
        return false;
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::getRefSpeeds(const int n_joint, const int *joints, double *spds)
{
    CD_DEBUG("(%d)\n", n_joint);
    if (!indexWithinRange(n_joint))
        return false;
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::getRefAccelerations(const int n_joint, const int *joints, double *accs)
{
    CD_DEBUG("(%d)\n", n_joint);
    if (!indexWithinRange(n_joint))
        return false;
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::stop(const int n_joint, const int *joints)
{
    CD_DEBUG("(%d)\n", n_joint);
    if (!indexWithinRange(n_joint))
        return false;
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::getTargetPosition(const int joint, double *ref)
{
    CD_DEBUG("(%d)\n", joint);
    if (!indexWithinRange(joint))
        return false;
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::getTargetPositions(double *refs)
{
    CD_DEBUG("\n");
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::getTargetPositions(const int n_joint, const int *joints, double *refs)
{
    CD_DEBUG("(%d)\n", n_joint);
    if (!indexWithinRange(n_joint))
        return false;
    return true;
}

// -----------------------------------------------------------------------------
