// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusControlboard.hpp"

#include <ColorDebug.h>

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool CanBusControlboard::setLimits(int axis, double min, double max)
{
    CD_DEBUG("(%d, %f, %f)\n", axis, min, max);
    CHECK_JOINT(axis);

    int localAxis;
    yarp::dev::IControlLimitsRaw * p = deviceMapper.getDevice(axis, &localAxis).iControlLimitsRaw;
    return p ? p->setLimitsRaw(localAxis, min, max) : false;
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::getLimits(int axis, double * min, double * max)
{
    CD_DEBUG("(%d)\n", axis);
    CHECK_JOINT(axis);

    int localAxis;
    yarp::dev::IControlLimitsRaw * p = deviceMapper.getDevice(axis, &localAxis).iControlLimitsRaw;
    return p ? p->getLimitsRaw(localAxis, min, max) : false;
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::setVelLimits(int axis, double min, double max)
{
    CD_DEBUG("(%d)\n", axis);
    CHECK_JOINT(axis);

    int localAxis;
    yarp::dev::IControlLimitsRaw * p = deviceMapper.getDevice(axis, &localAxis).iControlLimitsRaw;
    return p ? p->setVelLimitsRaw(localAxis, min, max) : false;
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::getVelLimits(int axis, double * min, double * max)
{
    CD_DEBUG("(%d)\n", axis);
    CHECK_JOINT(axis);

    int localAxis;
    yarp::dev::IControlLimitsRaw * p = deviceMapper.getDevice(axis, &localAxis).iControlLimitsRaw;
    return p ? p->getVelLimitsRaw(localAxis, min, max) : false;
}

// -----------------------------------------------------------------------------
