// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __IPOS_STATE_VARIABLES_HPP__
#define __IPOS_STATE_VARIABLES_HPP__

#include <cstdint>

#include <atomic>
#include <bitset>
#include <memory>
#include <mutex>
#include <string>

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
    EncoderRead(std::int32_t initialPos);
    void update(std::int32_t newPos, double newTime = 0.0);
    void reset(std::int32_t pos = 0);
    std::int32_t queryPosition() const;
    double querySpeed() const;
    double queryAcceleration() const;
    double queryTime() const;

private:
    std::int32_t lastPosition, nextToLastPosition;
    double lastSpeed, nextToLastSpeed;
    double lastAcceleration;
    yarp::os::Stamp lastStamp;
    mutable std::mutex encoderMutex;
};

/**
 * @ingroup TechnosoftIpos
 * @brief ...
 */
class StateVariables
{
public:

    StateVariables();

    bool validateInitialState(unsigned int canId);

    bool awaitControlMode(yarp::conf::vocab32_t mode);

    template<std::size_t N>
    void reportBitToggle(const std::string & msg, const std::bitset<N> & actual, const std::bitset<N> & stored, std::size_t pos)
    { if (actual.test(pos) != stored.test(pos)) reportBitToggleInternal(msg, actual.test(pos)); }

    double degreesToInternalUnits(double value, int derivativeOrder = 0);

    double internalUnitsToDegrees(double value, int derivativeOrder = 0);

    std::int16_t currentToInternalUnits(double value);

    double internalUnitsToCurrent(std::int16_t value);

    double internalUnitsToPeakCurrent(std::int16_t value);

    double currentToTorque(double current);

    double torqueToCurrent(double torque);

    std::unique_ptr<StateObserver> controlModeObserverPtr;

    // read/write, no concurrent access

    std::bitset<16> msr;
    std::bitset<16> mer;
    std::bitset<16> der;
    std::bitset<16> der2;
    std::bitset<16> cer;

    // read/write, those require atomic access

    EncoderRead lastEncoderRead;
    std::atomic<std::int16_t> lastCurrentRead;

    std::atomic<yarp::conf::vocab32_t> actualControlMode;
    std::atomic<yarp::conf::vocab32_t> requestedcontrolMode;

    std::atomic<double> tr;
    std::atomic<double> k;
    std::atomic<int> encoderPulses;

    // read only, conceptually immutable

    double drivePeakCurrent;
    double maxVel;

    std::string axisName;
    yarp::conf::vocab32_t jointType;

    bool reverse;

    // read only, fresh values queried from iPOS drive

    double min;
    double max;
    double refSpeed;
    double refAcceleration;

    int pulsesPerSample;

private:

    void reportBitToggleInternal(const std::string & msg, bool isSet);

    unsigned int canId;
};

} // namespace roboticslab

#endif // __IPOS_STATE_VARIABLES_HPP__
