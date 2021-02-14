// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __LINEAR_INTERPOLATION_BUFFER_HPP__
#define __LINEAR_INTERPOLATION_BUFFER_HPP__

#include <cstdint>

#include <deque>
#include <mutex>
#include <string>
#include <vector>

#include <yarp/os/Searchable.h>

#include "StateVariables.hpp"

namespace roboticslab
{

/**
 * @ingroup TechnosoftIpos
 * @brief Base class for a PT/PVT buffer of setpoints.
 *
 * Stores an internal queue of setpoints aimed to be processed in batches on
 * demand by client code.
 */
class LinearInterpolationBuffer
{
public:
    //! Constructor, sets internal invariable parameters.
    LinearInterpolationBuffer(const StateVariables & vars);

    //! Virtual destructor.
    virtual ~LinearInterpolationBuffer() = default;

    //! Store initial position.
    void setInitial(double initialTarget);

    //! Get buffer type as string identifier (pt/pvt).
    virtual std::string getType() const = 0;

    //! Get PT/PVT buffer size.
    virtual std::uint16_t getBufferSize() const = 0;

    //! Get buffer configuration (object 2074h).
    std::uint16_t getBufferConfig() const;

    //! Generate interpolation submode register value (object 60C0h).
    virtual std::int16_t getSubMode() const = 0;

    //! Place a new setpoint at the end of the queue.
    void addSetpoint(double target);

    //! Generate next batch of setpoints popped from the front of the queue.
    std::vector<std::uint64_t> popBatch(bool fullBuffer);

    //! Retrieve last point loaded into the buffer.
    double getPrevTarget() const;

    //! Report whether there are enough points in the queue to fill the buffer.
    bool isQueueReady() const;

    //! Report whether there are no more points in the queue.
    bool isQueueEmpty() const;

protected:
    using ip_record = std::pair<double, double>;

    //! Retrieve current integrity counter value.
    std::uint8_t getIntegrityCounter() const;

    //! Determine how many points should be left in the queue on each non-final batch update.
    virtual std::size_t getOffset() const;

    //! Obtain time in UI samples for current segment, update internal counters.
    std::uint16_t getSampledTime(double currentTimestamp);

    //! Generate interpolation data record given three contiguous position target (object 60C1h).
    virtual std::uint64_t makeDataRecord(const ip_record & previous, const ip_record & current, const ip_record & next) = 0;

    const StateVariables & vars;

private:
    std::uint8_t integrityCounter;
    ip_record prevTarget;
    double initialTimestamp;
    int sampleCount;
    std::deque<ip_record> pendingTargets;
    mutable std::mutex queueMutex;
};

/**
 * @ingroup TechnosoftIpos
 * @brief Implementation of a PT buffer.
 */
class PtBuffer : public LinearInterpolationBuffer
{
public:
    using LinearInterpolationBuffer::LinearInterpolationBuffer;

    std::string getType() const override;
    std::uint16_t getBufferSize() const override;
    std::int16_t getSubMode() const override;

protected:
    std::uint64_t makeDataRecord(const ip_record & previous, const ip_record & current, const ip_record & next) override;
};

/**
 * @ingroup TechnosoftIpos
 * @brief Implementation of a PVT buffer.
 *
 * By using the lasts two position targets, this class computes the mean
 * velocity target required by the linear interpolator.
 */
class PvtBuffer : public LinearInterpolationBuffer
{
public:
    using LinearInterpolationBuffer::LinearInterpolationBuffer;

    std::string getType() const override;
    std::uint16_t getBufferSize() const override;
    std::int16_t getSubMode() const override;

protected:
    std::size_t getOffset() const override;
    std::uint64_t makeDataRecord(const ip_record & previous, const ip_record & current, const ip_record & next) override;
};

//! Factory method.
LinearInterpolationBuffer * createInterpolationBuffer(const yarp::os::Searchable & config, const StateVariables & vars);

} // namespace roboticslab

#endif // __LINEAR_INTERPOLATION_BUFFER_HPP__
