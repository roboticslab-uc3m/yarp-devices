// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TechnosoftIpos.hpp"

#include <cmath>

#include <yarp/conf/numeric.h>
#include <yarp/os/Log.h>

#include "LogComponent.hpp"

using namespace roboticslab;

// ----------------------------------------------------------------------------------

bool TechnosoftIpos::velocityMoveRaw(int j, double sp)
{
    yCTrace(IPOS, "%d %f", j, sp);
    CHECK_JOINT(j);
    CHECK_MODE(VOCAB_CM_VELOCITY);

    const auto maxVel = vars.maxVel.load();

    if (std::abs(sp) > maxVel)
    {
        yCWarning(IPOS, "Requested speed exceeds maximum velocity (%f)", maxVel);
        sp = yarp::conf::clamp(sp, -maxVel, maxVel);
    }

    // reset halt bit
    if (can->driveStatus()->controlword()[8]
        && !can->driveStatus()->controlword(can->driveStatus()->controlword().reset(8)))
    {
        return false;
    }

    vars.synchronousCommandTarget = sp;
    return true;
}

// ----------------------------------------------------------------------------------

bool TechnosoftIpos::velocityMoveRaw(const double * sp)
{
    return velocityMoveRaw(0, sp[0]);
}

// ----------------------------------------------------------------------------------

bool TechnosoftIpos::velocityMoveRaw(int n_joint, const int * joints, const double * spds)
{
    return velocityMoveRaw(joints[0], spds[0]);
}

// -----------------------------------------------------------------------------

bool TechnosoftIpos::getRefVelocityRaw(int joint, double * vel)
{
    yCTrace(IPOS, "%d",joint);
    CHECK_JOINT(joint);
    CHECK_MODE(VOCAB_CM_VELOCITY);
    *vel = vars.synchronousCommandTarget;
    return true;
}

// ------------------------------------------------------------------------------

bool TechnosoftIpos::getRefVelocitiesRaw(double * vels)
{
    return getRefVelocityRaw(0, &vels[0]);
}

// -----------------------------------------------------------------------------

bool TechnosoftIpos::getRefVelocitiesRaw(int n_joint, const int * joints, double * vels)
{
    return getRefVelocityRaw(joints[0], &vels[0]);
}

// -----------------------------------------------------------------------------
