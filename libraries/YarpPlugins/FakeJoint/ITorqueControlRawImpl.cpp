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

bool roboticslab::FakeJoint::getBemfParamRaw(int j, double *bemf)
{
    CD_INFO("\n");
    return true;
}

// ------------------------------------------------------------------------------

bool roboticslab::FakeJoint::setBemfParamRaw(int j, double bemf)
{
    CD_INFO("\n");
    return true;
}

// ------------------------------------------------------------------------------

bool roboticslab::FakeJoint::setTorquePidRaw(int j, const yarp::dev::Pid &pid)
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

bool roboticslab::FakeJoint::setTorquePidsRaw(const yarp::dev::Pid *pids)
{
    CD_ERROR("\n");
    return false;
}

// ------------------------------------------------------------------------------

bool roboticslab::FakeJoint::setTorqueErrorLimitRaw(int j, double limit)
{
    CD_INFO("\n");
    return true;
}

// ------------------------------------------------------------------------------

bool roboticslab::FakeJoint::setTorqueErrorLimitsRaw(const double *limits)
{
    CD_ERROR("\n");
    return false;
}

// ------------------------------------------------------------------------------

bool roboticslab::FakeJoint::getTorqueErrorRaw(int j, double *err)
{
    CD_INFO("\n");
    return true;
}

// ------------------------------------------------------------------------------

bool roboticslab::FakeJoint::getTorqueErrorsRaw(double *errs)
{
    CD_ERROR("\n");
    return false;
}

// ------------------------------------------------------------------------------

bool roboticslab::FakeJoint::getTorquePidOutputRaw(int j, double *out)
{
    CD_INFO("\n");
    return true;
}

// ------------------------------------------------------------------------------

bool roboticslab::FakeJoint::getTorquePidOutputsRaw(double *outs)
{
    CD_ERROR("\n");
    return false;
}

// ------------------------------------------------------------------------------

bool roboticslab::FakeJoint::getTorquePidRaw(int j, yarp::dev::Pid *pid)
{
    CD_INFO("\n");
    return true;
}

// ------------------------------------------------------------------------------

bool roboticslab::FakeJoint::getTorquePidsRaw(yarp::dev::Pid *pids)
{
    CD_ERROR("\n");
    return false;
}

// ------------------------------------------------------------------------------

bool roboticslab::FakeJoint::getTorqueErrorLimitRaw(int j, double *limit)
{
    CD_INFO("\n");
    return true;
}

// ------------------------------------------------------------------------------

bool roboticslab::FakeJoint::getTorqueErrorLimitsRaw(double *limits)
{
    CD_ERROR("\n");
    return false;
}

// ------------------------------------------------------------------------------

bool roboticslab::FakeJoint::resetTorquePidRaw(int j)
{
    CD_INFO("\n");
    return true;
}

// ------------------------------------------------------------------------------

bool roboticslab::FakeJoint::disableTorquePidRaw(int j)
{
    CD_INFO("\n");
    return true;
}

// ------------------------------------------------------------------------------

bool roboticslab::FakeJoint::enableTorquePidRaw(int j)
{
    CD_INFO("\n");
    return true;
}

// ------------------------------------------------------------------------------

bool roboticslab::FakeJoint::setTorqueOffsetRaw(int j, double v)
{
    CD_INFO("\n");
    return true;
}
