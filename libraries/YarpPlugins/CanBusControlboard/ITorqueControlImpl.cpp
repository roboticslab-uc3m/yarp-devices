// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusControlboard.hpp"

#include <ColorDebug.h>

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool CanBusControlboard::getRefTorque(int j, double * t)
{
    CD_DEBUG("(%d)\n", j);
    CHECK_JOINT(j);
    return deviceMapper.singleJointMapping(j, t, &yarp::dev::ITorqueControlRaw::getRefTorqueRaw);
}

bool CanBusControlboard::getRefTorques(double * t)
{
    CD_DEBUG("\n");
    return deviceMapper.fullJointMapping(t, &yarp::dev::ITorqueControlRaw::getRefTorquesRaw);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::setRefTorque(int j, double t)
{
    CD_DEBUG("(%d, %f)\n", j, t);
    CHECK_JOINT(j);
    return deviceMapper.singleJointMapping(j, t, &yarp::dev::ITorqueControlRaw::setRefTorqueRaw);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::setRefTorques(const double * t)
{
    CD_DEBUG("\n");
    return deviceMapper.fullJointMapping(t, &yarp::dev::ITorqueControlRaw::setRefTorquesRaw);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::setRefTorques(int n_joint, const int * joints, const double * t)
{
    CD_DEBUG("\n");
    return deviceMapper.multiJointMapping(n_joint, joints, t, &yarp::dev::ITorqueControlRaw::setRefTorquesRaw);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::getMotorTorqueParams(int j,  yarp::dev::MotorTorqueParameters * params)
{
    CD_DEBUG("(%d)\n", j);
    CHECK_JOINT(j);
    return deviceMapper.singleJointMapping(j, params, &yarp::dev::ITorqueControlRaw::getMotorTorqueParamsRaw);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::setMotorTorqueParams(int j, const yarp::dev::MotorTorqueParameters params)
{
    CD_DEBUG("(%d)\n", j);
    CHECK_JOINT(j);
    return deviceMapper.singleJointMapping(j, params, &yarp::dev::ITorqueControlRaw::setMotorTorqueParamsRaw);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::getTorque(int j, double * t)
{
    //CD_DEBUG("(%d)\n",j); //-- Too verbose in controlboardwrapper2 stream.
    CHECK_JOINT(j);
    return deviceMapper.singleJointMapping(j, t, &yarp::dev::ITorqueControlRaw::getTorqueRaw);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::getTorques(double * t)
{
    CD_DEBUG("\n");
    return deviceMapper.fullJointMapping(t, &yarp::dev::ITorqueControlRaw::getTorquesRaw);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::getTorqueRange(int j, double * min, double * max)
{
    CD_DEBUG("(%d)\n", j);
    CHECK_JOINT(j);

    int localAxis;
    yarp::dev::ITorqueControlRaw * p = deviceMapper.getDevice(j, &localAxis).iTorqueControlRaw;
    return p ? p->getTorqueRangeRaw(localAxis, min, max) : false;
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::getTorqueRanges(double * mins, double * maxs)
{
    CD_DEBUG("\n");

    const int * localAxisOffsets;
    const std::vector<RawDevice> & rawDevices = deviceMapper.getDevices(localAxisOffsets);

    bool ok = true;

    for (int i = 0; i < rawDevices.size(); i++)
    {
        yarp::dev::ITorqueControlRaw * p = rawDevices[i].iTorqueControlRaw;
        ok &= p ? p->getTorqueRangesRaw(mins + localAxisOffsets[i], maxs + localAxisOffsets[i]) : false;
    }

    return ok;
}

// -----------------------------------------------------------------------------
