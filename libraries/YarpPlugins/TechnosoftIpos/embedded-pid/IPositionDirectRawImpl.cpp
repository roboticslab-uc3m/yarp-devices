// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "embedded-pid/TechnosoftIposEmbedded.hpp"

#include <cmath> // std::abs, std::copysign

#include <yarp/os/Log.h>
#include <yarp/os/SystemClock.h>

#include "LogComponent.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue TechnosoftIposEmbedded::setPositionRaw(int j, double ref)
#else
bool TechnosoftIposEmbedded::setPositionRaw(int j, double ref)
#endif
{
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
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
            return ok ? yarp::dev::ReturnValue::return_code::return_value_ok : yarp::dev::ReturnValue::return_code::return_value_error_method_failed;
#else
            return ok;
#endif
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

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue::return_code::return_value_ok;
#else
    return true;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue TechnosoftIposEmbedded::getRefPositionRaw(int joint, double * ref)
#else
bool TechnosoftIposEmbedded::getRefPositionRaw(int joint, double * ref)
#endif
{
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

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue::return_code::return_value_ok;
#else
    return true;
#endif
}

// -----------------------------------------------------------------------------
