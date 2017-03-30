// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "FakeJoint.hpp"

// ######################## ITorqueControlRaw Related ##########################

bool teo::FakeJoint::setTorqueModeRaw()
{
    CD_ERROR("\n");
    return false;
}

bool teo::FakeJoint::getRefTorquesRaw(double *t)
{
    CD_ERROR("\n");
    return false;
}

bool teo::FakeJoint::getRefTorqueRaw(int j, double *t)
{
    CD_INFO("\n");
    return true;
}

bool teo::FakeJoint::setRefTorquesRaw(const double *t)
{
    CD_ERROR("\n");
    return false;
}

bool teo::FakeJoint::setRefTorqueRaw(int j, double t)
{
    CD_INFO("\n");
    return true;
}

bool teo::FakeJoint::getBemfParamRaw(int j, double *bemf)
{
    CD_INFO("\n");
    return true;
}

bool teo::FakeJoint::setBemfParamRaw(int j, double bemf)
{
    CD_INFO("\n");
    return true;
}

bool teo::FakeJoint::setTorquePidRaw(int j, const yarp::dev::Pid &pid)
{
    CD_INFO("\n");
    return true;
}

bool teo::FakeJoint::getTorqueRaw(int j, double *t)
{
    //CD_INFO("\n");  //-- Too verbose in controlboardwrapper2 stream.
    return true;
}

bool teo::FakeJoint::getTorquesRaw(double *t)
{
    CD_ERROR("\n");
    return false;
}

bool teo::FakeJoint::getTorqueRangeRaw(int j, double *min, double *max)
{
    CD_INFO("\n");
    return true;
}

bool teo::FakeJoint::getTorqueRangesRaw(double *min, double *max)
{
    CD_ERROR("\n");
    return false;
}

bool teo::FakeJoint::setTorquePidsRaw(const yarp::dev::Pid *pids)
{
    CD_ERROR("\n");
    return false;
}

bool teo::FakeJoint::setTorqueErrorLimitRaw(int j, double limit)
{
    CD_INFO("\n");
    return true;
}

bool teo::FakeJoint::setTorqueErrorLimitsRaw(const double *limits)
{
    CD_ERROR("\n");
    return false;
}

bool teo::FakeJoint::getTorqueErrorRaw(int j, double *err)
{
    CD_INFO("\n");
    return true;
}

bool teo::FakeJoint::getTorqueErrorsRaw(double *errs)
{
    CD_ERROR("\n");
    return false;
}

bool teo::FakeJoint::getTorquePidOutputRaw(int j, double *out)
{
    CD_INFO("\n");
    return true;
}

bool teo::FakeJoint::getTorquePidOutputsRaw(double *outs)
{
    CD_ERROR("\n");
    return false;
}

bool teo::FakeJoint::getTorquePidRaw(int j, yarp::dev::Pid *pid)
{
    CD_INFO("\n");
    return true;
}

bool teo::FakeJoint::getTorquePidsRaw(yarp::dev::Pid *pids)
{
    CD_ERROR("\n");
    return false;
}

bool teo::FakeJoint::getTorqueErrorLimitRaw(int j, double *limit)
{
    CD_INFO("\n");
    return true;
}

bool teo::FakeJoint::getTorqueErrorLimitsRaw(double *limits)
{
    CD_ERROR("\n");
    return false;
}

bool teo::FakeJoint::resetTorquePidRaw(int j)
{
    CD_INFO("\n");
    return true;
}

bool teo::FakeJoint::disableTorquePidRaw(int j)
{
    CD_INFO("\n");
    return true;
}

bool teo::FakeJoint::enableTorquePidRaw(int j)
{
    CD_INFO("\n");
    return true;
}

bool teo::FakeJoint::setTorqueOffsetRaw(int j, double v)
{
    CD_INFO("\n");
    return true;
}
