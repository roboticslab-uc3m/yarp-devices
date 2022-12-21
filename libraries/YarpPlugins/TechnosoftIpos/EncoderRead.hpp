// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __ENCODER_READ_HPP__
#define __ENCODER_READ_HPP__

#include <cstdint>

#include <mutex>

#include <yarp/os/Stamp.h>

namespace roboticslab
{

/**
 * @ingroup TechnosoftIpos
 * @brief Stores last encoder reads, obtains mean speeds and accelerations
 * via differentiation.
 */
class EncoderRead
{
public:
    //! Constructor.
    EncoderRead(double samplingPeriod);

    //! Set new position (counts), update speeds (counts/sample) and accelerations (counts/sample^2).
    void update(std::int32_t newPos);

    //! Reset internals to zero, pick provided position (encoder counts).
    void reset(std::int32_t pos = 0);

    //! Retrieve current position (encoder counts).
    std::int32_t queryPosition() const;

    //! Retrieve instantaneous speed (encoder counts/sample).
    double querySpeed() const;

    //! Retrieve instantaneous acceleration (encoder counts/sample^2).
    double queryAcceleration() const;

    //! Retrieve last timestamp (seconds).
    double queryTime() const;

private:
    const double samplingFreq; // samples per second
    std::int32_t lastPosition;
    double lastSpeed;
    double lastAcceleration;
    yarp::os::Stamp lastStamp;
    mutable std::mutex encoderMutex;
};

} // namespace roboticslab

#endif // __ENCODER_READ_HPP__
