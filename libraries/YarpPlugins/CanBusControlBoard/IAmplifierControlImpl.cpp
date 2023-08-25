// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusControlBoard.hpp"

#include <yarp/os/Log.h>

#include "LogComponent.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool CanBusControlBoard::enableAmp(int j)
{
    yCTrace(CBCB, "%d", j);
    CHECK_JOINT(j);
    return deviceMapper.mapSingleJoint(&yarp::dev::IAmplifierControlRaw::enableAmpRaw, j);
}

// -----------------------------------------------------------------------------

bool CanBusControlBoard::disableAmp(int j)
{
    yCTrace(CBCB, "%d", j);
    CHECK_JOINT(j);
    return deviceMapper.mapSingleJoint(&yarp::dev::IAmplifierControlRaw::disableAmpRaw, j);
}

// -----------------------------------------------------------------------------

bool CanBusControlBoard::getAmpStatus(int j, int * v)
{
    yCTrace(CBCB, "%d", j);
    CHECK_JOINT(j);
    return deviceMapper.mapSingleJoint(&yarp::dev::IAmplifierControlRaw::getAmpStatusRaw, j, v);
}

// -----------------------------------------------------------------------------

bool CanBusControlBoard::getAmpStatus(int * st)
{
    yCTrace(CBCB, "");
    return deviceMapper.mapAllJoints(&yarp::dev::IAmplifierControlRaw::getAmpStatusRaw, st);
}

// -----------------------------------------------------------------------------

bool CanBusControlBoard::getMaxCurrent(int j, double * v)
{
    yCTrace(CBCB, "%d", j);
    CHECK_JOINT(j);
    return deviceMapper.mapSingleJoint(&yarp::dev::IAmplifierControlRaw::getMaxCurrentRaw, j, v);
}

// -----------------------------------------------------------------------------

bool CanBusControlBoard::setMaxCurrent(int j, double v)
{
    yCTrace(CBCB, "%d %f", j, v);
    CHECK_JOINT(j);
    return deviceMapper.mapSingleJoint(&yarp::dev::IAmplifierControlRaw::setMaxCurrentRaw, j, v);
}

// -----------------------------------------------------------------------------

bool CanBusControlBoard::getNominalCurrent(int m, double * val)
{
    yCTrace(CBCB, "%d", m);
    CHECK_JOINT(m);
    return deviceMapper.mapSingleJoint(&yarp::dev::IAmplifierControlRaw::getNominalCurrentRaw, m, val);
}

// -----------------------------------------------------------------------------

bool CanBusControlBoard::setNominalCurrent(int m, double val)
{
    yCTrace(CBCB, "%d %f", m, val);
    CHECK_JOINT(m);
    return deviceMapper.mapSingleJoint(&yarp::dev::IAmplifierControlRaw::setNominalCurrentRaw, m, val);
}

// -----------------------------------------------------------------------------

bool CanBusControlBoard::getPeakCurrent(int m, double * val)
{
    yCTrace(CBCB, "%d", m);
    CHECK_JOINT(m);
    return deviceMapper.mapSingleJoint(&yarp::dev::IAmplifierControlRaw::getPeakCurrentRaw, m, val);
}

// -----------------------------------------------------------------------------

bool CanBusControlBoard::setPeakCurrent(int m, double val)
{
    yCTrace(CBCB, "%d %f", m, val);
    CHECK_JOINT(m);
    return deviceMapper.mapSingleJoint(&yarp::dev::IAmplifierControlRaw::setPeakCurrentRaw, m, val);
}

// -----------------------------------------------------------------------------

bool CanBusControlBoard::getPWM(int j, double * val)
{
    yCTrace(CBCB, "%d", j);
    CHECK_JOINT(j);
    return deviceMapper.mapSingleJoint(&yarp::dev::IAmplifierControlRaw::getPWMRaw, j, val);
}

// -----------------------------------------------------------------------------

bool CanBusControlBoard::getPWMLimit(int j, double * val)
{
    yCTrace(CBCB, "%d", j);
    CHECK_JOINT(j);
    return deviceMapper.mapSingleJoint(&yarp::dev::IAmplifierControlRaw::getPWMLimitRaw, j, val);
}

// -----------------------------------------------------------------------------

bool CanBusControlBoard::setPWMLimit(int j, double val)
{
    yCTrace(CBCB, "%d %f", j, val);
    CHECK_JOINT(j);
    return deviceMapper.mapSingleJoint(&yarp::dev::IAmplifierControlRaw::setPWMLimitRaw, j, val);
}

// -----------------------------------------------------------------------------

bool CanBusControlBoard::getPowerSupplyVoltage(int j, double * val)
{
    yCTrace(CBCB, "%d", j);
    CHECK_JOINT(j);
    return deviceMapper.mapSingleJoint(&yarp::dev::IAmplifierControlRaw::getPowerSupplyVoltageRaw, j, val);
}

// -----------------------------------------------------------------------------
