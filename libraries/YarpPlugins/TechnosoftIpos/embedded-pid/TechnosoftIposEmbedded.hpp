// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __TECHNOSOFT_IPOS_EMBEDDED_HPP__
#define __TECHNOSOFT_IPOS_EMBEDDED_HPP__

#include <atomic>
#include <bitset>

#include "TechnosoftIposBase.hpp"
#include "embedded-pid/InterpolatedPositionBuffer.hpp"

namespace roboticslab
{

/**
 * @ingroup TechnosoftIpos
 * @brief A TechnosoftIposBase implementation using the firmware-embedded PID.
 */
class TechnosoftIposEmbedded : public TechnosoftIposBase
{
public:
    using TechnosoftIposBase::TechnosoftIposBase;

    //  --------- DeviceDriver declarations. Implementation in DeviceDriverImpl.cpp ---------

    bool open(yarp::os::Searchable & config) override;
    bool close() override;

    //  --------- ICanBusSharer declarations. Implementation in ICanBusSharerImpl.cpp ---------

    bool synchronize(double timestamp) override;

    //  --------- IControlModeRaw declarations. Implementation in IControlModeRawImpl.cpp ---------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return_t getAvailableControlModesRaw(int j, std::vector<yarp::dev::SelectableControlModeEnum> & avail) override;
    return_t getControlModeRaw(int j, yarp::dev::ControlModeEnum & mode) override;
    return_t setControlModeRaw(int j, yarp::dev::SelectableControlModeEnum mode) override;
#else
    return_t getControlModeRaw(int j, int * mode) override;
    return_t setControlModeRaw(int j, int mode) override;
#endif

    // ------- IPositionControlRaw declarations. Implementation in IPositionControlRawImpl.cpp -------

    return_t positionMoveRaw(int j, double ref) override;
    return_t relativeMoveRaw(int j, double delta) override;
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return_t checkMotionDoneRaw(int j, bool & flag) override;
    return_t setTrajSpeedRaw(int j, double sp) override;
    return_t setTrajAccelerationRaw(int j, double acc) override;
    return_t getTrajSpeedRaw(int j, double * ref) override;
    return_t getTrajAccelerationRaw(int j, double * acc) override;
#else
    return_t checkMotionDoneRaw(int j, bool * flag) override;
    return_t setRefSpeedRaw(int j, double sp) override;
    return_t setRefAccelerationRaw(int j, double acc) override;
    return_t getRefSpeedRaw(int j, double * ref) override;
    return_t getRefAccelerationRaw(int j, double * acc) override;
#endif
    return_t stopRaw(int j) override;
    return_t getTargetPositionRaw(int joint, double * ref) override;

    // ------- IPositionDirectRaw declarations. Implementation in IPositionDirectRawImpl.cpp -------

    return_t setPositionRaw(int j, double ref) override;
    return_t getRefPositionRaw(int joint, double * ref) override;

    // ------- IRemoteVariablesRaw declarations. Implementation in IRemoteVariablesRawImpl.cpp -------

    return_t getRemoteVariableRaw(std::string key, yarp::os::Bottle & val) override;
    return_t setRemoteVariableRaw(std::string key, const yarp::os::Bottle & val) override;
    return_t getRemoteVariablesListRaw(yarp::os::Bottle * listOfKeys) override;

    //  --------- IVelocityControlRaw declarations. Implementation in IVelocityControlRawImpl.cpp ---------

    return_t velocityMoveRaw(int j, double sp) override;
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return_t getTargetVelocityRaw(int joint, double * vel) override;
#else
    return_t getRefVelocityRaw(int joint, double * vel) override;
#endif

private:
    void interpretModesOfOperation(std::int8_t modesOfOperation) override;
    void interpretIpStatus(std::uint16_t ipStatus) override;
    void onPositionLimitTriggered() override;
    void reset() override;

    InterpolatedPositionBuffer * ipBuffer {nullptr};
    std::string ipMode;
    int ipPeriodMs {0};

    std::bitset<16> ipStatus;

    std::atomic<bool> ipMotionStarted {false};
    std::atomic<bool> ipBufferFilled {false};
    std::atomic<bool> ipBufferEnabled {false};

    std::atomic<bool> enableSync {false};
    std::atomic<bool> enableCsv {false};

    std::atomic<double> targetPosition {0.0};
    std::atomic<double> targetVelocity {0.0};
};

} // namespace roboticslab

#endif // __TECHNOSOFT_IPOS_EMBEDDED_HPP__
