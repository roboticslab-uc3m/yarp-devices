// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TechnosoftIpos.hpp"

#include <cmath> // std::abs, std::copysign

#include <yarp/os/Log.h>
#include <yarp/os/SystemClock.h>

#include "LogComponent.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool TechnosoftIpos::setPositionRaw(int j, double ref)
{
    yCITrace(IPOS, id(), "%d %f", j, ref);
    CHECK_JOINT(j);
    CHECK_MODE(VOCAB_CM_POSITION_DIRECT);

    if (ipBuffer)
    {
        ipBuffer->addSetpoint(ref); // register point in the internal queue

        // ip mode is enabled, drive's buffer is empty, motion has not started yet, we have enough points in the queue
        if (vars.ipBufferEnabled && !vars.ipBufferFilled && !vars.ipMotionStarted && ipBuffer->isQueueReady())
        {
            std::int32_t refInternal = vars.lastEncoderRead->queryPosition();
            ipBuffer->setInitial(vars.internalUnitsToDegrees(refInternal));

            bool ok = true;

            for (auto setpoint : ipBuffer->popBatch(true))
            {
                ok &= can->rpdo3()->write(setpoint); // load point into the buffer
            }

            vars.ipBufferFilled = ok;
            return ok;
        }
    }
    else
    {
        double previousTimestamp;
        double currentTimestamp = yarp::os::SystemClock::nowSystem();
        double previousRef = commandBuffer.getStoredCommand(&previousTimestamp);
        double diff = ref - previousRef;
        double period = currentTimestamp - previousTimestamp;
        double velocity = std::abs(diff / period);
        double maxVel = vars.maxVel.load();

        if (velocity > maxVel)
        {
            double newRef = previousRef + std::copysign(maxVel * period, diff);
            yCIWarning(IPOS, id(), "Maximum velocity exceeded (%f > %f), clipping reference from %f to %f", velocity, maxVel, ref, newRef);
            ref = newRef;
        }

        commandBuffer.accept(ref);
    }

    return true;
}
// -----------------------------------------------------------------------------

bool TechnosoftIpos::setPositionsRaw(const double * refs)
{
    return setPositionRaw(0, refs[0]);
}

// -----------------------------------------------------------------------------

bool TechnosoftIpos::setPositionsRaw(int n_joint, const int * joints, const double * refs)
{
    return setPositionRaw(joints[0], refs[0]);
}

// -----------------------------------------------------------------------------

bool TechnosoftIpos::getRefPositionRaw(int joint, double * ref)
{
    yCITrace(IPOS, id(), "%d", joint);
    CHECK_JOINT(joint);
    CHECK_MODE(VOCAB_CM_POSITION_DIRECT);

    if (ipBuffer)
    {
        *ref = ipBuffer->getPrevTarget();
    }
    else
    {
        *ref = commandBuffer.getStoredCommand();
    }

    return true;
}

// -----------------------------------------------------------------------------

bool TechnosoftIpos::getRefPositionsRaw(double * refs)
{
    return getRefPositionRaw(0, &refs[0]);
}

// -----------------------------------------------------------------------------

bool TechnosoftIpos::getRefPositionsRaw(int n_joint, const int * joints, double * refs)
{
    return getRefPositionRaw(joints[0], &refs[0]);
}

// -----------------------------------------------------------------------------
