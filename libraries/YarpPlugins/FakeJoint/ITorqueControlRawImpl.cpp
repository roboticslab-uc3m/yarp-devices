// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "FakeJoint.hpp"

// ######################## ITorqueControlRaw Related ##########################

bool roboticslab::FakeJoint::getRefTorquesRaw(double *t)
{
    CD_ERROR("\n");
    return false;
}

// ------------------------------------------------------------------------------

bool roboticslab::FakeJoint::getRefTorqueRaw(int j, double *t)
{
    CD_INFO("\n");
    return true;
}

// ------------------------------------------------------------------------------

bool roboticslab::FakeJoint::setRefTorquesRaw(const double *t)
{
    CD_ERROR("\n");
    return false;
}

// ------------------------------------------------------------------------------

bool roboticslab::FakeJoint::setRefTorqueRaw(int j, double t)
{
    CD_INFO("\n");
    return true;
}

// ------------------------------------------------------------------------------

bool roboticslab::FakeJoint::getTorqueRaw(int j, double *t)
{
    //CD_INFO("\n");  //-- Too verbose in controlboardwrapper2 stream.
    return true;
}

// ------------------------------------------------------------------------------

bool roboticslab::FakeJoint::getTorquesRaw(double *t)
{
    CD_ERROR("\n");
    return false;
}

// ------------------------------------------------------------------------------

bool roboticslab::FakeJoint::getTorqueRangeRaw(int j, double *min, double *max)
{
    CD_INFO("\n");
    return true;
}

// ------------------------------------------------------------------------------

bool roboticslab::FakeJoint::getTorqueRangesRaw(double *min, double *max)
{
    CD_ERROR("\n");
    return false;
}

// ------------------------------------------------------------------------------
