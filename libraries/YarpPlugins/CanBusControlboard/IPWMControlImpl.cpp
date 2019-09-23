// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusControlboard.hpp"

#include <ColorDebug.h>

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool CanBusControlboard::setRefDutyCycle(int m, double ref)
{
    CD_DEBUG("(%d, %f)\n", m, ref);
    CHECK_JOINT(m);
    return deviceMapper.singleJointMapping(m, ref, &yarp::dev::IPWMControlRaw::setRefDutyCycleRaw);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::setRefDutyCycles(const double * refs)
{
    CD_DEBUG("\n");
    return deviceMapper.fullJointMapping(refs, &yarp::dev::IPWMControlRaw::setRefDutyCyclesRaw);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::getRefDutyCycle(int m, double * ref)
{
    CD_DEBUG("(%d)\n", m);
    CHECK_JOINT(m);
    return deviceMapper.singleJointMapping(m, ref, &yarp::dev::IPWMControlRaw::getRefDutyCycleRaw);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::getRefDutyCycles(double * refs)
{
    CD_DEBUG("\n");
    return deviceMapper.fullJointMapping(refs, &yarp::dev::IPWMControlRaw::getRefDutyCyclesRaw);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::getDutyCycle(int m, double * val)
{
    CD_DEBUG("(%d)\n", m);
    CHECK_JOINT(m);
    return deviceMapper.singleJointMapping(m, val, &yarp::dev::IPWMControlRaw::getDutyCycleRaw);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::getDutyCycles(double * vals)
{
    CD_DEBUG("\n");
    return deviceMapper.fullJointMapping(vals, &yarp::dev::IPWMControlRaw::getDutyCyclesRaw);
}

// -----------------------------------------------------------------------------
