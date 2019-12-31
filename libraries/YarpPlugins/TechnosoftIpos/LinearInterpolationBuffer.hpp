// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __LINEAR_INTERPOLATION_BUFFER_HPP__
#define __LINEAR_INTERPOLATION_BUFFER_HPP__

#include <cstdint>

#include <mutex>
#include <string>

#include <yarp/os/Searchable.h>

#include "StateVariables.hpp"

#define DEFAULT_PERIOD_MS 50
#define DEFAULT_BUFFER_SIZE 1
#define DEFAULT_MODE "pt"

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
 * Stores two setpoints: the last that was sent to the drive, and the next that
 * awaits to be sent. It is designed so that the current motor position can be
 * continuously commanded by a periodic thread governing this class.
 * Occasional updates requested by an external trajectory generator would be
 * registered for command on the next iteration.
 */
class LinearInterpolationBuffer
{
public:
    //! Constructor, set internal invariable parameters.
    LinearInterpolationBuffer(int _periodMs, int _bufferSize, double _factor, double _maxVel);

    //! Virtual destructor.
    virtual ~LinearInterpolationBuffer() = default;

    //! Set integrity counter to zero.
    void resetIntegrityCounter();

    //! Register first position target to be sent to the drive.
    virtual void setInitialReference(double target);

    //! Get buffer type as string identifier (pt/pvt).
    virtual std::string getType() const = 0;

    //! Register the next position target to be sent to the drive.
    void updateTarget(double target);

    //! Retrieve the last position target that was sent to the drive.
    double getLastTarget() const;

    //! Get PT/PVT mode period.
    int getPeriod() const;

    //! Set new PT/PVT mode period.
    void setPeriod(int periodMs);

    //! Get PT/PVT buffer size.
    int getBufferSize() const;

    //! Set new PT/PVT buffer size.
    void setBufferSize(int bufferSize);

    //! Generate interpolation submode register value (object 60C0h).
    virtual std::int16_t getSubMode() const = 0;

    //! Generate interpolation data record (object 60C1h).
    virtual std::uint64_t makeDataRecord() = 0;

    //! Clone this instance to the selected buffer type class.
    LinearInterpolationBuffer * cloneTo(const std::string & type);

    //! Factory method.
    static LinearInterpolationBuffer * createBuffer(const yarp::os::Searchable & config, const StateVariables & vars);

protected:
    int periodMs;
    int bufferSize;
    double factor;
    double maxVel;
    double lastSentTarget;
    double lastReceivedTarget;
    int integrityCounter;
    mutable std::mutex mutex;
};

/**
 * @ingroup TechnosoftIpos
 * @brief Implementation of a PT buffer.
 */
class PtBuffer : public LinearInterpolationBuffer
{
public:
    //! Constructor, sets the max distance allowed to travel given a max velocity.
    PtBuffer(int _periodMs, int _bufferSize, double _factor, double _maxVel);

    virtual std::string getType() const override;
    virtual std::int16_t getSubMode() const override;
    virtual std::uint64_t makeDataRecord() override;

private:
    double maxDistance;
};

/**
 * @ingroup TechnosoftIpos
 * @brief Implementation of a PVT buffer.
 *
 * In contrast to PtBuffer, this class stores the last two reference setpoints in
 * order to compute the mean velocity target required by the linear interpolator.
 */
class PvtBuffer : public LinearInterpolationBuffer
{
public:
    //! Constructor.
    PvtBuffer(int _periodMs, int _bufferSize, double _factor, double _maxVel);

    virtual void setInitialReference(double target) override;
    virtual std::string getType() const override;
    virtual std::int16_t getSubMode() const override;
    virtual std::uint64_t makeDataRecord() override;

private:
    double previousTarget;
    bool isFirstPoint;
};

} // namespace roboticslab

#endif // __LINEAR_INTERPOLATION_BUFFER_HPP__
