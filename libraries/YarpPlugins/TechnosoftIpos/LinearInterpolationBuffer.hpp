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
    LinearInterpolationBuffer(const StateVariables & vars, std::uint16_t periodMs);

    //! Virtual destructor.
    virtual ~LinearInterpolationBuffer() = default;

    //! Set integrity counter to zero and clear internal setpoint queue.
    void init(double initialTarget);

    //! Get buffer type as string identifier (pt/pvt).
    virtual std::string getType() const = 0;

    //! Get PT/PVT mode period.
    std::uint16_t getPeriodMs() const;

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
    //! Retrieve current integrity counter value.
    std::uint8_t getIntegrityCounter() const;

    //! Generate interpolation data record given three contiguous position target (object 60C1h).
    virtual std::uint64_t makeDataRecord(double previous, double current, double next) = 0;

    const StateVariables & vars;

    static const unsigned int PT_BUFFER_MAX;
    static const unsigned int PVT_BUFFER_MAX;
    static const unsigned int BUFFER_LOW;

private:
    std::uint16_t periodMs;
    std::uint8_t integrityCounter;
    double prevTarget;

    std::deque<double> pendingTargets;
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

    virtual std::string getType() const override;
    virtual std::uint16_t getBufferSize() const override;
    virtual std::int16_t getSubMode() const override;

protected:
    virtual std::uint64_t makeDataRecord(double previous, double current, double next) override;
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

    virtual std::string getType() const override;
    virtual std::uint16_t getBufferSize() const override;
    virtual std::int16_t getSubMode() const override;

protected:
    virtual std::uint64_t makeDataRecord(double previous, double current, double next) override;
};

//! Factory method.
LinearInterpolationBuffer * createInterpolationBuffer(const yarp::os::Searchable & config, const StateVariables & vars);

} // namespace roboticslab

#endif // __LINEAR_INTERPOLATION_BUFFER_HPP__
