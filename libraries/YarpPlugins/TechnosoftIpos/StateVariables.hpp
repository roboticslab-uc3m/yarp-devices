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

#include "PdoProtocol.hpp"
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
struct StateVariables
{
    StateVariables();

    bool validateInitialState(unsigned int canId);

    bool awaitControlMode(yarp::conf::vocab32_t mode);

    double degreesToInternalUnits(double value, int derivativeOrder = 0) const;

    double internalUnitsToDegrees(double value, int derivativeOrder = 0) const;

    std::int16_t currentToInternalUnits(double value) const;

    double internalUnitsToCurrent(std::int16_t value) const;

    double internalUnitsToPeakCurrent(std::int16_t value) const;

    double currentToTorque(double current) const;

    double torqueToCurrent(double torque) const;

    std::unique_ptr<StateObserver> controlModeObserverPtr;

    // read/write, no concurrent access

    std::bitset<16> msr;
    std::bitset<16> mer;
    std::bitset<16> der;
    std::bitset<16> der2;
    std::bitset<16> cer;
    std::bitset<16> ptStatus;
    std::int8_t modesOfOperation;

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

    PdoConfiguration tpdo1Conf;
    PdoConfiguration tpdo2Conf;
    PdoConfiguration tpdo3Conf;

    // read only, fresh values queried from iPOS drive

    double min;
    double max;
    double refSpeed;
    double refAcceleration;

    int pulsesPerSample;
};

} // namespace roboticslab

#endif // __IPOS_STATE_VARIABLES_HPP__
