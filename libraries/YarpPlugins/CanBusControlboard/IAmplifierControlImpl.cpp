// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusControlboard.hpp"

#include <yarp/os/Log.h>

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool CanBusControlboard::enableAmp(int j)
{
    yTrace("%d", j);
    CHECK_JOINT(j);
    return deviceMapper.mapSingleJoint(&yarp::dev::IAmplifierControlRaw::enableAmpRaw, j);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::disableAmp(int j)
{
    yTrace("%d", j);
    CHECK_JOINT(j);
    return deviceMapper.mapSingleJoint(&yarp::dev::IAmplifierControlRaw::disableAmpRaw, j);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::getAmpStatus(int j, int * v)
{
    yTrace("%d", j);
    CHECK_JOINT(j);
    return deviceMapper.mapSingleJoint(&yarp::dev::IAmplifierControlRaw::getAmpStatusRaw, j, v);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::getAmpStatus(int * st)
{
    yTrace("");
    return deviceMapper.mapAllJoints(&yarp::dev::IAmplifierControlRaw::getAmpStatusRaw, st);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::getMaxCurrent(int j, double * v)
{
    yTrace("%d", j);
    CHECK_JOINT(j);
    return deviceMapper.mapSingleJoint(&yarp::dev::IAmplifierControlRaw::getMaxCurrentRaw, j, v);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::setMaxCurrent(int j, double v)
{
    yTrace("%d %f", j, v);
    CHECK_JOINT(j);
    return deviceMapper.mapSingleJoint(&yarp::dev::IAmplifierControlRaw::setMaxCurrentRaw, j, v);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::getNominalCurrent(int m, double * val)
{
    yTrace("%d", m);
    CHECK_JOINT(m);
    return deviceMapper.mapSingleJoint(&yarp::dev::IAmplifierControlRaw::getNominalCurrentRaw, m, val);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::setNominalCurrent(int m, double val)
{
    yTrace("%d %f", m, val);
    CHECK_JOINT(m);
    return deviceMapper.mapSingleJoint(&yarp::dev::IAmplifierControlRaw::setNominalCurrentRaw, m, val);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::getPeakCurrent(int m, double * val)
{
    yTrace("%d", m);
    CHECK_JOINT(m);
    return deviceMapper.mapSingleJoint(&yarp::dev::IAmplifierControlRaw::getPeakCurrentRaw, m, val);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::setPeakCurrent(int m, double val)
{
    yTrace("%d %f", m, val);
    CHECK_JOINT(m);
    return deviceMapper.mapSingleJoint(&yarp::dev::IAmplifierControlRaw::setPeakCurrentRaw, m, val);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::getPWM(int j, double * val)
{
    yTrace("%d", j);
    CHECK_JOINT(j);
    return deviceMapper.mapSingleJoint(&yarp::dev::IAmplifierControlRaw::getPWMRaw, j, val);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::getPWMLimit(int j, double * val)
{
    yTrace("%d", j);
    CHECK_JOINT(j);
    return deviceMapper.mapSingleJoint(&yarp::dev::IAmplifierControlRaw::getPWMLimitRaw, j, val);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::setPWMLimit(int j, double val)
{
    yTrace("%d %f", j, val);
    CHECK_JOINT(j);
    return deviceMapper.mapSingleJoint(&yarp::dev::IAmplifierControlRaw::setPWMLimitRaw, j, val);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::getPowerSupplyVoltage(int j, double * val)
{
    yTrace("%d", j);
    CHECK_JOINT(j);
    return deviceMapper.mapSingleJoint(&yarp::dev::IAmplifierControlRaw::getPowerSupplyVoltageRaw, j, val);
}

// -----------------------------------------------------------------------------
