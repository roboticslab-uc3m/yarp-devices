// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusControlboard.hpp"

#include <ColorDebug.h>

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool CanBusControlboard::enableAmp(int j)
{
    CD_DEBUG("(%d)\n", j);
    CHECK_JOINT(j);

    int localAxis;
    yarp::dev::IAmplifierControlRaw * p = deviceMapper.getDevice(j, &localAxis).iAmplifierControlRaw;
    return p ? p->enableAmpRaw(localAxis) : false;
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::disableAmp(int j)
{
    CD_DEBUG("(%d)\n", j);
    CHECK_JOINT(j);

    int localAxis;
    yarp::dev::IAmplifierControlRaw * p = deviceMapper.getDevice(j, &localAxis).iAmplifierControlRaw;
    return p ? p->disableAmpRaw(localAxis) : false;
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::getAmpStatus(int j, int * v)
{
    CD_DEBUG("(%d)\n", j);
    CHECK_JOINT(j);
    return deviceMapper.singleJointMapping(j, v, &yarp::dev::IAmplifierControlRaw::getAmpStatusRaw);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::getAmpStatus(int * st)
{
    CD_DEBUG("\n");
    return deviceMapper.fullJointMapping(st, &yarp::dev::IAmplifierControlRaw::getAmpStatusRaw);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::getMaxCurrent(int j, double * v)
{
    CD_DEBUG("(%d)\n", j);
    CHECK_JOINT(j);
    return deviceMapper.singleJointMapping(j, v, &yarp::dev::IAmplifierControlRaw::getMaxCurrentRaw);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::setMaxCurrent(int j, double v)
{
    CD_DEBUG("(%d, %f)\n", j, v);
    CHECK_JOINT(j);
    return deviceMapper.singleJointMapping(j, v, &yarp::dev::IAmplifierControlRaw::setMaxCurrentRaw);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::getNominalCurrent(int m, double * val)
{
    CD_DEBUG("(%d)\n", m);
    CHECK_JOINT(m);
    return deviceMapper.singleJointMapping(m, val, &yarp::dev::IAmplifierControlRaw::getNominalCurrentRaw);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::setNominalCurrent(int m, double val)
{
    CD_DEBUG("(%d, %f)\n", m, val);
    CHECK_JOINT(m);
    return deviceMapper.singleJointMapping(m, val, &yarp::dev::IAmplifierControlRaw::setNominalCurrentRaw);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::getPeakCurrent(int m, double * val)
{
    CD_DEBUG("(%d)\n", m);
    CHECK_JOINT(m);
    return deviceMapper.singleJointMapping(m, val, &yarp::dev::IAmplifierControlRaw::getPeakCurrentRaw);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::setPeakCurrent(int m, double val)
{
    CD_DEBUG("(%d, %f)\n", m, val);
    CHECK_JOINT(m);
    return deviceMapper.singleJointMapping(m, val, &yarp::dev::IAmplifierControlRaw::setPeakCurrentRaw);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::getPWM(int j, double * val)
{
    CD_DEBUG("(%d)\n", j);
    CHECK_JOINT(j);
    return deviceMapper.singleJointMapping(j, val, &yarp::dev::IAmplifierControlRaw::getPWMRaw);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::getPWMLimit(int j, double * val)
{
    CD_DEBUG("(%d)\n", j);
    CHECK_JOINT(j);
    return deviceMapper.singleJointMapping(j, val, &yarp::dev::IAmplifierControlRaw::getPWMLimitRaw);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::setPWMLimit(int j, double val)
{
    CD_DEBUG("(%d, %f)\n", j, val);
    CHECK_JOINT(j);
    return deviceMapper.singleJointMapping(j, val, &yarp::dev::IAmplifierControlRaw::setPWMLimitRaw);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::getPowerSupplyVoltage(int j, double * val)
{
    CD_DEBUG("(%d)\n", j);
    CHECK_JOINT(j);
    return deviceMapper.singleJointMapping(j, val, &yarp::dev::IAmplifierControlRaw::getPowerSupplyVoltageRaw);
}

// -----------------------------------------------------------------------------
