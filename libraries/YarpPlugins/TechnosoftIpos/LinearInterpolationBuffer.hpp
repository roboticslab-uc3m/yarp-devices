// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __LINEAR_INTERPOLATION_BUFFER_HPP__
#define __LINEAR_INTERPOLATION_BUFFER_HPP__

#include <cstdint>

#include <atomic>
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
 * It is designed so that the current motor position can be continuously
 * commanded by a periodic thread governing this class.
 */
class LinearInterpolationBuffer
{
public:
    //! Constructor, set internal invariable parameters.
    LinearInterpolationBuffer(const StateVariables & vars, std::uint16_t periodMs);

    //! Virtual destructor.
    virtual ~LinearInterpolationBuffer() = default;

    //! Set integrity counter to zero and clear internal setpoint queue.
    void reset();

    //! Get buffer type as string identifier (pt/pvt).
    virtual std::string getType() const = 0;

    //! Get PT/PVT mode period.
    std::uint16_t getPeriodMs() const;

    //! Get PT/PVT buffer size.
    virtual std::uint16_t getBufferSize() const = 0;

    //! Get size of internal setpoint queue.
    unsigned int getQueueSize() const;

    //! Get buffer configuration (object 2074h).
    std::uint16_t getBufferConfig() const;

    //! Generate interpolation submode register value (object 60C0h).
    virtual std::int16_t getSubMode() const = 0;

    //! Place a new setpoint at the end of the queue.
    void addSetpoint(double target);

    //! Generate next batch of setpoints popped from the front of the queue.
    std::vector<std::uint64_t> popBatch(bool fullBuffer);

    //! Report whether motion has started.
    bool isStarted() const;

    //! Update motion status.
    void reportMotionStatus(bool isStarted);

    //! Factory method.
    static LinearInterpolationBuffer * createBuffer(const yarp::os::Searchable & config,
            const StateVariables & vars);

protected:
    //! Retrieve current integrity counter value.
    std::uint8_t getIntegrityCounter() const;

    //! Generate interpolation data record given a position target (object 60C1h).
    virtual std::uint64_t makeDataRecord(double target) = 0;

    const StateVariables & vars;
    std::uint16_t periodMs;
    std::deque<std::uint64_t> pendingSetpoints;
    mutable std::mutex queueMutex;

    static const unsigned int PT_BUFFER_MAX;
    static const unsigned int PVT_BUFFER_MAX;
    static const unsigned int BUFFER_LOW;

private:
    std::uint8_t integrityCounter;
    std::atomic<bool> motionStarted;
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
    virtual std::uint64_t makeDataRecord(double target) override;
};

/**
 * @ingroup TechnosoftIpos
 * @brief Implementation of a PVT buffer.
 *
 * In contrast to @ref PtBuffer, this class stores the last reference setpoint in
 * order to compute the mean velocity target required by the linear interpolator.
 */
class PvtBuffer : public LinearInterpolationBuffer
{
public:
    using LinearInterpolationBuffer::LinearInterpolationBuffer;

    virtual std::string getType() const override;
    virtual std::uint16_t getBufferSize() const override;
    virtual std::int16_t getSubMode() const override;

protected:
    virtual std::uint64_t makeDataRecord(double target) override;

private:
    double previousTarget = 0.0;
    bool isFirstPoint = true;
};

} // namespace roboticslab

#endif // __LINEAR_INTERPOLATION_BUFFER_HPP__
