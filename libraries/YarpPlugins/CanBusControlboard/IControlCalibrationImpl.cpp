// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusControlboard.hpp"

#include <ColorDebug.h>

using namespace roboticslab;
using raw_t = yarp::dev::IControlCalibrationRaw;

// -----------------------------------------------------------------------------

bool CanBusControlboard::calibrateAxisWithParams(int axis, unsigned int type, double p1, double p2, double p3)
{
    CD_DEBUG("(%d, %d, %f, %f, %f)\n", axis, type, p1, p2, p3);
    CHECK_JOINT(axis);
    auto fn = &yarp::dev::IControlCalibrationRaw::calibrateAxisWithParamsRaw;
    return deviceMapper.mapSingleJoint(fn, axis, type, p1, p2, p3);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::setCalibrationParameters(int axis, const yarp::dev::CalibrationParameters & params)
{
    CD_DEBUG("(%d)\n", axis);
    CHECK_JOINT(axis);
    auto fn = &yarp::dev::IControlCalibrationRaw::setCalibrationParametersRaw;
    return deviceMapper.mapSingleJoint<raw_t, const yarp::dev::CalibrationParameters &>(fn, axis, params);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::calibrationDone(int j)
{
    CD_DEBUG("(%d)\n", j);
    CHECK_JOINT(j);
    return deviceMapper.mapSingleJoint(&yarp::dev::IControlCalibrationRaw::calibrationDoneRaw, j);
}

// -----------------------------------------------------------------------------
