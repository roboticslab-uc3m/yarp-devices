// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusBroker.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue CanBusBroker::calibrateAxisWithParams(int axis, unsigned int type, double p1, double p2, double p3)
#else
bool CanBusBroker::calibrateAxisWithParams(int axis, unsigned int type, double p1, double p2, double p3)
#endif
{
    CHECK_JOINT(axis);
    return deviceMapper.mapSingleJoint(&yarp::dev::IControlCalibrationRaw::calibrateAxisWithParamsRaw, axis, type, p1, p2, p3);
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue CanBusBroker::setCalibrationParameters(int axis, const yarp::dev::CalibrationParameters & params)
#else
bool CanBusBroker::setCalibrationParameters(int axis, const yarp::dev::CalibrationParameters & params)
#endif
{
    CHECK_JOINT(axis);
    return deviceMapper.mapSingleJoint(&yarp::dev::IControlCalibrationRaw::setCalibrationParametersRaw, axis, params);
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue CanBusBroker::calibrationDone(int j)
#else
bool CanBusBroker::calibrationDone(int j)
#endif
{
    CHECK_JOINT(j);
    return deviceMapper.mapSingleJoint(&yarp::dev::IControlCalibrationRaw::calibrationDoneRaw, j);
}

// -----------------------------------------------------------------------------
