// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusControlboard.hpp"

#include <ColorDebug.h>

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool CanBusControlboard::getImpedance(int j, double * stiffness, double * damping)
{
    CD_DEBUG("(%d)\n", j);
    CHECK_JOINT(j);

    int localAxis;
    yarp::dev::IImpedanceControlRaw * p = deviceMapper.getDevice(j, &localAxis).iImpedanceControlRaw;
    return p ? p->getImpedanceRaw(localAxis, stiffness, damping) : false;
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::setImpedance(int j, double stiffness, double damping)
{
    CD_DEBUG("(%d, %f, %f)\n", j, stiffness, damping);
    CHECK_JOINT(j);

    int localAxis;
    yarp::dev::IImpedanceControlRaw * p = deviceMapper.getDevice(j, &localAxis).iImpedanceControlRaw;
    return p ? p->setImpedanceRaw(localAxis, stiffness, damping) : false;
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::setImpedanceOffset(int j, double offset)
{
    CD_DEBUG("(%d, %f)\n", j, offset);
    CHECK_JOINT(j);
    return deviceMapper.singleJointMapping(j, offset, &yarp::dev::IImpedanceControlRaw::setImpedanceOffsetRaw);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::getImpedanceOffset(int j, double * offset)
{
    CD_DEBUG("(%d)\n", j);
    CHECK_JOINT(j);
    return deviceMapper.singleJointMapping(j, offset, &yarp::dev::IImpedanceControlRaw::getImpedanceOffsetRaw);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::getCurrentImpedanceLimit(int j, double * min_stiff, double * max_stiff, double * min_damp,
        double * max_damp)
{
    CD_DEBUG("(%d)\n", j);
    CHECK_JOINT(j);

    int localAxis;
    yarp::dev::IImpedanceControlRaw * p = deviceMapper.getDevice(j, &localAxis).iImpedanceControlRaw;
    return p ? p->getCurrentImpedanceLimitRaw(localAxis, min_stiff, max_stiff, min_damp, max_damp) : false;
}

// -----------------------------------------------------------------------------
