// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __IPOS_STATE_VARIABLES_HPP__
#define __IPOS_STATE_VARIABLES_HPP__

#include <cstdint>

#include <initializer_list>
#include <memory>
#include <mutex>

#include <yarp/conf/numeric.h>
#include <yarp/os/Stamp.h>

#include "StateObserver.hpp"

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
    EncoderRead(double initialPos);
    void update(double newPos, double newTime = 0.0);
    void reset(double pos = 0.0);
    double queryPosition() const;
    double querySpeed() const;
    double queryAcceleration() const;
    double queryTime() const;

private:
    double lastPosition, nextToLastPosition;
    double lastSpeed, nextToLastSpeed;
    double lastAcceleration;
    yarp::os::Stamp lastStamp;
    mutable std::mutex encoderMutex;
};

/**
 * @ingroup TechnosoftIpos
 * @brief ...
 */
struct StateVariables
{
    StateVariables();

    bool validateInitialState();

    bool awaitControlMode(yarp::conf::vocab32_t mode);

    bool expectControlModes(std::initializer_list<yarp::conf::vocab32_t> modes);

    std::int32_t degreesToInternalUnits(double value, int derivativeOrder = 0);

    double internalUnitsToDegrees(std::int32_t value, int derivativeOrder = 0);

    std::int16_t currentToInternalUnits(double value);

    double internalUnitsToCurrent(std::int16_t value);

    double internalUnitsToPeakCurrent(std::int16_t value);

    double currentToTorque(double current);

    double torqueToCurrent(double torque);

    EncoderRead lastEncoderRead;

    yarp::conf::vocab32_t actualControlMode;
    yarp::conf::vocab32_t requestedcontrolMode;

    double drivePeakCurrent;
    double maxVel;
    double tr;
    double k;

    double min;
    double max;
    double refSpeed;
    double refAcceleration;

    int encoderPulses;
    int pulsesPerSample;

    std::unique_ptr<StateObserver> controlModeObserverPtr;
};

} // namespace roboticslab

#endif // __IPOS_STATE_VARIABLES_HPP__