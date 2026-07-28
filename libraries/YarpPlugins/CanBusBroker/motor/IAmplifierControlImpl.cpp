// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusBroker.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool CanBusBroker::enableAmp(int j)
{
    CHECK_JOINT(j);
    return deviceMapper.mapSingleJoint(&yarp::dev::IAmplifierControlRaw::enableAmpRaw, j);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::disableAmp(int j)
{
    CHECK_JOINT(j);
    return deviceMapper.mapSingleJoint(&yarp::dev::IAmplifierControlRaw::disableAmpRaw, j);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::getAmpStatus(int j, int * v)
{
    CHECK_JOINT(j);
    return deviceMapper.mapSingleJoint(&yarp::dev::IAmplifierControlRaw::getAmpStatusRaw, j, v);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::getAmpStatus(int * st)
{
    return deviceMapper.mapAllJoints(&yarp::dev::IAmplifierControlRaw::getAmpStatusRaw, st);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::getMaxCurrent(int j, double * v)
{
    CHECK_JOINT(j);
    return deviceMapper.mapSingleJoint(&yarp::dev::IAmplifierControlRaw::getMaxCurrentRaw, j, v);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::setMaxCurrent(int j, double v)
{
    CHECK_JOINT(j);
    return deviceMapper.mapSingleJoint(&yarp::dev::IAmplifierControlRaw::setMaxCurrentRaw, j, v);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::getNominalCurrent(int m, double * val)
{
    CHECK_JOINT(m);
    return deviceMapper.mapSingleJoint(&yarp::dev::IAmplifierControlRaw::getNominalCurrentRaw, m, val);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::setNominalCurrent(int m, double val)
{
    CHECK_JOINT(m);
    return deviceMapper.mapSingleJoint(&yarp::dev::IAmplifierControlRaw::setNominalCurrentRaw, m, val);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::getPeakCurrent(int m, double * val)
{
    CHECK_JOINT(m);
    return deviceMapper.mapSingleJoint(&yarp::dev::IAmplifierControlRaw::getPeakCurrentRaw, m, val);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::setPeakCurrent(int m, double val)
{
    CHECK_JOINT(m);
    return deviceMapper.mapSingleJoint(&yarp::dev::IAmplifierControlRaw::setPeakCurrentRaw, m, val);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::getPWM(int j, double * val)
{
    CHECK_JOINT(j);
    return deviceMapper.mapSingleJoint(&yarp::dev::IAmplifierControlRaw::getPWMRaw, j, val);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::getPWMLimit(int j, double * val)
{
    CHECK_JOINT(j);
    return deviceMapper.mapSingleJoint(&yarp::dev::IAmplifierControlRaw::getPWMLimitRaw, j, val);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::setPWMLimit(int j, double val)
{
    CHECK_JOINT(j);
    return deviceMapper.mapSingleJoint(&yarp::dev::IAmplifierControlRaw::setPWMLimitRaw, j, val);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::getPowerSupplyVoltage(int j, double * val)
{
    CHECK_JOINT(j);
    return deviceMapper.mapSingleJoint(&yarp::dev::IAmplifierControlRaw::getPowerSupplyVoltageRaw, j, val);
}

// -----------------------------------------------------------------------------
