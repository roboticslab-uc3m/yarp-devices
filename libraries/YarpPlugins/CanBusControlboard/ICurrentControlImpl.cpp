// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusControlboard.hpp"

#include <ColorDebug.h>

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool CanBusControlboard::getNumberOfMotors(int * ax)
{
    CD_DEBUG("\n");
    return getAxes(ax);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::getCurrent(int m, double * curr)
{
    CD_DEBUG("(%d)\n", m);
    CHECK_JOINT(m);
    return deviceMapper.singleJointMapping(m, curr, &yarp::dev::ICurrentControlRaw::getCurrentRaw);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::getCurrents(double * currs)
{
    CD_DEBUG("\n");
    return deviceMapper.fullJointMapping(currs, &yarp::dev::ICurrentControlRaw::getCurrentsRaw);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::getCurrentRange(int m, double * min, double * max)
{
    CD_DEBUG("(%d)\n", m);
    CHECK_JOINT(m);

    int localAxis;
    yarp::dev::ICurrentControlRaw * p = deviceMapper.getDevice(m, &localAxis).iCurrentControlRaw;
    return p ? p->getCurrentRangeRaw(localAxis, min, max) : false;
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::getCurrentRanges(double * mins, double * maxs)
{
    CD_DEBUG("\n");

    const int * localAxisOffsets;
    const std::vector<RawDevice> & rawDevices = deviceMapper.getDevices(localAxisOffsets);

    bool ok = true;

    for (int i = 0; i < rawDevices.size(); i++)
    {
        yarp::dev::ICurrentControlRaw * p = rawDevices[i].iCurrentControlRaw;
        ok &= p ? p->getCurrentRangesRaw(mins + localAxisOffsets[i], maxs + localAxisOffsets[i]) : false;
    }

    return ok;
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::setRefCurrent(int m, double curr)
{
    CD_DEBUG("(%d)\n", m);
    CHECK_JOINT(m);
    return deviceMapper.singleJointMapping(m, curr, &yarp::dev::ICurrentControlRaw::setRefCurrentRaw);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::setRefCurrents(const double * currs)
{
    CD_DEBUG("\n");
    return deviceMapper.fullJointMapping(currs, &yarp::dev::ICurrentControlRaw::setRefCurrentsRaw);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::setRefCurrents(int n_motor, const int * motors, const double * currs)
{
    CD_DEBUG("(%d)\n", n_motor);
    return deviceMapper.multiJointMapping(n_motor, motors, currs, &yarp::dev::ICurrentControlRaw::setRefCurrentsRaw);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::getRefCurrent(int m, double * curr)
{
    CD_DEBUG("(%d)\n", m);
    CHECK_JOINT(m);
    return deviceMapper.singleJointMapping(m, curr, &yarp::dev::ICurrentControlRaw::getRefCurrentRaw);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::getRefCurrents(double * currs)
{
    CD_DEBUG("\n");
    return deviceMapper.fullJointMapping(currs, &yarp::dev::ICurrentControlRaw::getRefCurrentsRaw);
}

// -----------------------------------------------------------------------------
