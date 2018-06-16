// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "FakeControlboard.hpp"

#include <ColorDebug.h>

// ------------------- IPositionControl2 Related --------------------------------

bool roboticslab::FakeControlboard::positionMove(const int n_joint, const int *joints, const double *refs)
{
    CD_DEBUG("\n");
    // must implement mask!
    return positionMove(refs);
}

// -----------------------------------------------------------------------------

bool roboticslab::FakeControlboard::relativeMove(const int n_joint, const int *joints, const double *deltas)
{
    CD_DEBUG("\n");
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::FakeControlboard::checkMotionDone(const int n_joint, const int *joints, bool *flags)
{
    CD_DEBUG("\n");
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::FakeControlboard::setRefSpeeds(const int n_joint, const int *joints, const double *spds)
{
    CD_DEBUG("\n");
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::FakeControlboard::setRefAccelerations(const int n_joint, const int *joints, const double *accs)
{
    CD_DEBUG("\n");
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::FakeControlboard::getRefSpeeds(const int n_joint, const int *joints, double *spds)
{
    CD_DEBUG("\n");
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::FakeControlboard::getRefAccelerations(const int n_joint, const int *joints, double *accs)
{
    CD_DEBUG("\n");
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::FakeControlboard::stop(const int n_joint, const int *joints)
{
    CD_DEBUG("\n");
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::FakeControlboard::getTargetPosition(const int joint, double *ref)
{
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::FakeControlboard::getTargetPositions(double *refs)
{
    CD_DEBUG("\n");
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::FakeControlboard::getTargetPositions(const int n_joint, const int *joints, double *refs)
{
    CD_DEBUG("\n");
    return true;
}

// -----------------------------------------------------------------------------
