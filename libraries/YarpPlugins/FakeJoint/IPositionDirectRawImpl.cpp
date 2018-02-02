// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "FakeJoint.hpp"

// ######################## IPositionDirectRaw Related ##########################

bool roboticslab::FakeJoint::setPositionRaw(int j, double ref)
{
    CD_DEBUG("\n");
    this->positionMoveRaw(0,ref);
    return true;
}

// ------------------------------------------------------------------------------

bool roboticslab::FakeJoint::setPositionsRaw(const int n_joint, const int *joints, double *refs)
{
    CD_DEBUG("\n");
    this->positionMoveRaw(0,refs[0]);
    return true;
}

// ------------------------------------------------------------------------------

bool roboticslab::FakeJoint::setPositionsRaw(const double *refs)
{
    CD_DEBUG("\n");
    return true;
}

// ------------------------------------------------------------------------------
