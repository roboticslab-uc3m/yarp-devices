// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __INTERPOLATED_POSITION_BUFFER_HPP__
#define __INTERPOLATED_POSITION_BUFFER_HPP__

#include <cstdint>

#include <deque>
#include <mutex>
#include <vector>

namespace roboticslab
{

/**
 * @ingroup TechnosoftIpos
 * @brief Base class for a PT/PVT buffer of setpoints.
 *
 * Stores an internal queue of setpoints aimed to be processed in batches on
 * demand by client code.
 */
class InterpolatedPositionBuffer
{
public:
    //! Constructor, sets internal invariable parameters.
    InterpolatedPositionBuffer(double samplingPeriod, double interpolationPeriod);

    //! Virtual destructor.
    virtual ~InterpolatedPositionBuffer() = default;

    //! Store initial position (internal units).
    void setInitial(int initialTarget);

    //! Get PT/PVT period if fixed (synchronous), zero otherwise (asynchronous).
    int getPeriodMs() const;

    //! Get PT/PVT buffer size.
    virtual std::uint16_t getBufferSize() const = 0;

    //! Get buffer configuration (object 2074h).
    std::uint16_t getBufferConfig() const;

    //! Generate interpolation submode register value (object 60C0h).
    virtual std::int16_t getSubMode() const = 0;

    //! Place a new setpoint (internal units) at the end of the queue.
    void addSetpoint(int target);

    //! Generate next batch of setpoints popped from the front of the queue.
    std::vector<std::uint64_t> popBatch(bool fullBuffer);

    //! Retrieve last point loaded into the buffer (internal units).
    int getPrevTarget() const;

    //! Report whether there are enough points in the queue to fill the buffer.
    bool isQueueReady() const;

    //! Report whether there are no more points in the queue.
    bool isQueueEmpty() const;

    //! Clear internal queue.
    void clearQueue();

protected:
    using ip_record = std::pair<int, double>; // position (internal units), timestamp (seconds)

    //! Retrieve current integrity counter value.
    std::uint8_t getIntegrityCounter() const;

    //! Determine how many points should be left in the queue on each non-final batch update.
    virtual std::size_t getOffset() const;

    //! Obtain time samples (internal units) for current segment, update internal counters.
    std::uint16_t getSampledTime(double currentTimestamp);

    //! Compute mean velocity (internal units) between two setpoints.
    double getMeanVelocity(const ip_record & earliest, const ip_record & latest) const;

    //! Generate interpolation data record given three contiguous position target (object 60C1h).
    virtual std::uint64_t makeDataRecord(const ip_record & previous, const ip_record & current, const ip_record & next) = 0;

    const double samplingPeriod; // [s]

private:
    const std::uint16_t fixedSamples;
    std::uint8_t integrityCounter;
    ip_record prevTarget;
    double initialTimestamp;
    int sampleCount;
    std::deque<ip_record> pendingTargets;
    mutable std::mutex queueMutex;
};

/**
 * @ingroup TechnosoftIpos
 * @brief Implementation of a PT buffer (linear interpolation).
 */
class PtBuffer : public InterpolatedPositionBuffer
{
public:
    using InterpolatedPositionBuffer::InterpolatedPositionBuffer;

    std::uint16_t getBufferSize() const override;
    std::int16_t getSubMode() const override;

protected:
    std::uint64_t makeDataRecord(const ip_record & previous, const ip_record & current, const ip_record & next) override;
};

/**
 * @ingroup TechnosoftIpos
 * @brief Implementation of a PVT buffer (cubic interpolation).
 *
 * By using the lasts two position targets, this class computes the mean
 * velocity target required by the linear interpolator.
 */
class PvtBuffer : public InterpolatedPositionBuffer
{
public:
    using InterpolatedPositionBuffer::InterpolatedPositionBuffer;

    std::uint16_t getBufferSize() const override;
    std::int16_t getSubMode() const override;

protected:
    std::size_t getOffset() const override;
    std::uint64_t makeDataRecord(const ip_record & previous, const ip_record & current, const ip_record & next) override;
};

} // namespace roboticslab

#endif // __INTERPOLATED_POSITION_BUFFER_HPP__
