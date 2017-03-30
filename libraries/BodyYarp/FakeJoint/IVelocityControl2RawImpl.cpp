// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "FakeJoint.hpp"

// ######################## IVelocityControl Related ##########################

bool teo::FakeJoint::setVelocityModeRaw()
{
    CD_ERROR("\n");
    return false;
}

bool teo::FakeJoint::velocityMoveRaw(int j, double sp)
{
    CD_INFO("\n");
    return true;
}

bool teo::FakeJoint::velocityMoveRaw(const double *sp)
{
    CD_ERROR("\n");
    return false;
}
