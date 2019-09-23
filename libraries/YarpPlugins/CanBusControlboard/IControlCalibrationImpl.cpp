// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusControlboard.hpp"

#include <ColorDebug.h>

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool CanBusControlboard::calibrateAxisWithParams(int axis, unsigned int type, double p1, double p2, double p3)
{
    CD_DEBUG("(%d, %d, %f, %f, %f)\n", axis, type, p1, p2, p3);
    CHECK_JOINT(axis);

    int localAxis;
    yarp::dev::IControlCalibrationRaw * p = deviceMapper.getDevice(axis, &localAxis).iControlCalibrationRaw;
    return p ? p->calibrateAxisWithParamsRaw(localAxis, type, p1, p2, p3) : false;
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::setCalibrationParameters(int axis, const yarp::dev::CalibrationParameters & params)
{
    CD_DEBUG("(%d)\n", axis);
    CHECK_JOINT(axis);
    auto fn = &yarp::dev::IControlCalibrationRaw::setCalibrationParametersRaw;
    return deviceMapper.singleJointMapping<yarp::dev::IControlCalibrationRaw, const yarp::dev::CalibrationParameters &>(axis, params, fn);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::calibrationDone(int j)
{
    CD_DEBUG("(%d)\n", j);
    CHECK_JOINT(j);

    int localAxis;
    yarp::dev::IControlCalibrationRaw * p = deviceMapper.getDevice(j, &localAxis).iControlCalibrationRaw;
    return p ? p->calibrationDoneRaw(localAxis) : false;
}

// -----------------------------------------------------------------------------
