// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __LINEAR_INTERPOLATION_BUFFER_HPP__
#define __LINEAR_INTERPOLATION_BUFFER_HPP__

#include <cstdint>

#include <string>

#include <yarp/os/Searchable.h>

#include "StateVariables.hpp"

// https://github.com/roboticslab-uc3m/yarp-devices/issues/198#issuecomment-487279910
#define PT_BUFFER_MAX_SIZE 285
#define PVT_BUFFER_MAX_SIZE 222
#define PT_PVT_BUFFER_LOW_SIGNAL 15 // max: 15

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
    LinearInterpolationBuffer(const StateVariables & vars, int periodMs, int bufferSize);

    //! Virtual destructor.
    virtual ~LinearInterpolationBuffer() = default;

    //! Set integrity counter to zero.
    void resetIntegrityCounter();

    //! Get buffer type as string identifier (pt/pvt).
    virtual std::string getType() const = 0;

    //! Get PT/PVT mode period.
    int getPeriodMs() const;

    //! Get PT/PVT buffer size.
    int getBufferSize() const;

    //! Generate interpolation submode register value (object 60C0h).
    virtual std::int16_t getSubMode() const = 0;

    //! Generate interpolation data record given a position target (object 60C1h).
    virtual std::uint64_t makeDataRecord(double target) = 0;

    //! Factory method.
    static LinearInterpolationBuffer * createBuffer(const yarp::os::Searchable & config,
            const StateVariables & vars, unsigned int canId);

protected:
    const StateVariables & vars;
    int periodMs;
    int bufferSize;
    int integrityCounter;
};

/**
 * @ingroup TechnosoftIpos
 * @brief Implementation of a PT buffer.
 */
class PtBuffer : public LinearInterpolationBuffer
{
public:
    //! Constructor.
    PtBuffer(const StateVariables & vars, int periodMs, int bufferSize, unsigned int canId);

    virtual std::string getType() const override;
    virtual std::int16_t getSubMode() const override;
    virtual std::uint64_t makeDataRecord(double target) override;
};

/**
 * @ingroup TechnosoftIpos
 * @brief Implementation of a PVT buffer.
 *
 * In contrast to PtBuffer, this class stores the last reference setpoints in
 * order to compute the mean velocity target required by the linear interpolator.
 */
class PvtBuffer : public LinearInterpolationBuffer
{
public:
    //! Constructor.
    PvtBuffer(const StateVariables & vars, int periodMs, int bufferSize, unsigned int canId);

    virtual std::string getType() const override;
    virtual std::int16_t getSubMode() const override;
    virtual std::uint64_t makeDataRecord(double target) override;

private:
    double previousTarget;
    bool isFirstPoint;
};

} // namespace roboticslab

#endif // __LINEAR_INTERPOLATION_BUFFER_HPP__
