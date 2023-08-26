// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusControlBoard.hpp"

#include <yarp/os/Log.h>

#include "LogComponent.hpp"

using namespace roboticslab;
using raw_t = yarp::dev::IControlCalibrationRaw;

// -----------------------------------------------------------------------------

bool CanBusControlBoard::calibrateAxisWithParams(int axis, unsigned int type, double p1, double p2, double p3)
{
    yCTrace(CBCB, "%d %d %f %f %f", axis, type, p1, p2, p3);
    CHECK_JOINT(axis);
    auto fn = &yarp::dev::IControlCalibrationRaw::calibrateAxisWithParamsRaw;
    return deviceMapper.mapSingleJoint(fn, axis, type, p1, p2, p3);
}

// -----------------------------------------------------------------------------

bool CanBusControlBoard::setCalibrationParameters(int axis, const yarp::dev::CalibrationParameters & params)
{
    yCTrace(CBCB, "%d", axis);
    CHECK_JOINT(axis);
    auto fn = &yarp::dev::IControlCalibrationRaw::setCalibrationParametersRaw;
    return deviceMapper.mapSingleJoint<raw_t, const yarp::dev::CalibrationParameters &>(fn, axis, params);
}

// -----------------------------------------------------------------------------

bool CanBusControlBoard::calibrationDone(int j)
{
    yCTrace(CBCB, "%d", j);
    CHECK_JOINT(j);
    return deviceMapper.mapSingleJoint(&yarp::dev::IControlCalibrationRaw::calibrationDoneRaw, j);
}

// -----------------------------------------------------------------------------
