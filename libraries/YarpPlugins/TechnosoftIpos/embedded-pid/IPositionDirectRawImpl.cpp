// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "embedded-pid/TechnosoftIposEmbedded.hpp"

#include <cmath> // std::abs, std::copysign

#include <yarp/os/Log.h>
#include <yarp/os/SystemClock.h>

#include "LogComponent.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool TechnosoftIposEmbedded::setPositionRaw(int j, double ref)
{
    yCITrace(IPOS, id(), "%d %f", j, ref);
    CHECK_JOINT(j);
    CHECK_MODE(VOCAB_CM_POSITION_DIRECT);

    if (ipBuffer)
    {
        ipBuffer->addSetpoint(degreesToInternalUnits(ref)); // register point in the internal queue

        // ip mode is enabled, drive's buffer is empty, motion has not started yet, we have enough points in the queue
        if (ipBufferEnabled && !ipBufferFilled && !ipMotionStarted && ipBuffer->isQueueReady())
        {
            ipBuffer->setInitial(lastEncoderRead->queryPosition());

            bool ok = true;

            for (auto setpoint : ipBuffer->popBatch(true))
            {
                ok &= can->rpdo3()->write(setpoint); // load point into the buffer
            }

            ipBufferFilled = ok;
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
        double maxVel = this->maxVel;

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

bool TechnosoftIposEmbedded::setPositionsRaw(const double * refs)
{
    return setPositionRaw(0, refs[0]);
}

// -----------------------------------------------------------------------------

bool TechnosoftIposEmbedded::setPositionsRaw(int n_joint, const int * joints, const double * refs)
{
    return setPositionRaw(joints[0], refs[0]);
}

// -----------------------------------------------------------------------------

bool TechnosoftIposEmbedded::getRefPositionRaw(int joint, double * ref)
{
    yCITrace(IPOS, id(), "%d", joint);
    CHECK_JOINT(joint);
    CHECK_MODE(VOCAB_CM_POSITION_DIRECT);

    if (ipBuffer)
    {
        *ref = internalUnitsToDegrees(ipBuffer->getPrevTarget());
    }
    else
    {
        *ref = commandBuffer.getStoredCommand();
    }

    return true;
}

// -----------------------------------------------------------------------------

bool TechnosoftIposEmbedded::getRefPositionsRaw(double * refs)
{
    return getRefPositionRaw(0, &refs[0]);
}

// -----------------------------------------------------------------------------

bool TechnosoftIposEmbedded::getRefPositionsRaw(int n_joint, const int * joints, double * refs)
{
    return getRefPositionRaw(joints[0], &refs[0]);
}

// -----------------------------------------------------------------------------
