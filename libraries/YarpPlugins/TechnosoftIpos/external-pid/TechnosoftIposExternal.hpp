// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __TECHNOSOFT_IPOS_EXTERNAL_HPP__
#define __TECHNOSOFT_IPOS_EXTERNAL_HPP__

#include <atomic>
#include <mutex>

#include "TechnosoftIposBase.hpp"
#include "TrapezoidalTrajectory.hpp"

namespace roboticslab
{

/**
 * @ingroup TechnosoftIpos
 * @brief A TechnosoftIposBase implementation using an external (TEO board) PID.
 */
class TechnosoftIposExternal : public TechnosoftIposBase
{
public:
    using TechnosoftIposBase::TechnosoftIposBase;

    //  --------- DeviceDriver declarations. Implementation in DeviceDriverImpl.cpp ---------

    bool open(yarp::os::Searchable & config) override;
    bool close() override;

    //  --------- ICanBusSharer declarations. Implementation in ICanBusSharerImpl.cpp ---------

    bool initialize() override;
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

    //  ---------- IImpedanceControlRaw declarations. Implementation in IImpedanceControlRawImpl.cpp ----------

    return_t getImpedanceRaw(int j, double * stiffness, double * damping) override;
    return_t setImpedanceRaw(int j, double stiffness, double damping) override;
    return_t setImpedanceOffsetRaw(int j, double offset) override;
    return_t getImpedanceOffsetRaw(int j, double * offset) override;
    return_t getCurrentImpedanceLimitRaw(int j, double * min_stiff, double * max_stiff, double * min_damp, double * max_damp) override;

    //  ---------- IInteractionModeRaw declarations. Implementation in IInteractionModeRawImpl.cpp ----------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return_t getInteractionModeRaw(int axis, yarp::dev::InteractionModeEnum & mode) override;
#else
    return_t getInteractionModeRaw(int axis, yarp::dev::InteractionModeEnum * mode) override;
#endif
    return_t setInteractionModeRaw(int axis, yarp::dev::InteractionModeEnum mode) override;

    //  --------- IPidControlRaw declarations. Implementation in IPidControlRawImpl.cpp ---------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return_t getAvailablePidsRaw(int j, std::vector<yarp::dev::PidControlTypeEnum> & avail) override;
#endif
    return_t setPidRaw(const yarp::dev::PidControlTypeEnum & pidtype, int j, const yarp::dev::Pid & pid) override;
    return_t setPidReferenceRaw(const yarp::dev::PidControlTypeEnum & pidtype, int j, double ref) override;
    return_t setPidErrorLimitRaw(const yarp::dev::PidControlTypeEnum & pidtype, int j, double limit) override;
    return_t getPidErrorRaw(const yarp::dev::PidControlTypeEnum & pidtype, int j, double * err) override;
    return_t getPidOutputRaw(const yarp::dev::PidControlTypeEnum & pidtype, int j, double * out) override;
    return_t getPidRaw(const yarp::dev::PidControlTypeEnum & pidtype, int j, yarp::dev::Pid * pid) override;
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return_t getPidOffsetRaw(const yarp::dev::PidControlTypeEnum & pidtype, int j, double & v) override;
    return_t getPidFeedforwardRaw(const yarp::dev::PidControlTypeEnum & pidtype, int j, double & v) override;
    return_t getPidExtraInfoRaw(const yarp::dev::PidControlTypeEnum & pidtype, int j, yarp::dev::PidExtraInfo & info) override;
    return_t getPidExtraInfosRaw(const yarp::dev::PidControlTypeEnum & pidtype, std::vector<yarp::dev::PidExtraInfo> & info) override;
#endif
    return_t getPidReferenceRaw(const yarp::dev::PidControlTypeEnum & pidtype, int j, double * ref) override;
    return_t getPidErrorLimitRaw(const yarp::dev::PidControlTypeEnum & pidtype, int j, double * limit) override;
    return_t resetPidRaw(const yarp::dev::PidControlTypeEnum & pidtype, int j) override;
    return_t disablePidRaw(const yarp::dev::PidControlTypeEnum & pidtype, int j) override;
    return_t enablePidRaw(const yarp::dev::PidControlTypeEnum & pidtype, int j) override;
    return_t setPidOffsetRaw(const yarp::dev::PidControlTypeEnum & pidtype, int j, double v) override;
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return_t setPidFeedforwardRaw(const yarp::dev::PidControlTypeEnum & pidtype, int j, double v) override;
    return_t isPidEnabledRaw(const yarp::dev::PidControlTypeEnum & pidtype, int j, bool & enabled) override;
#else
    return_t isPidEnabledRaw(const yarp::dev::PidControlTypeEnum & pidtype, int j, bool * enabled) override;
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
    void onPositionLimitTriggered() override;
    void reset() override;

    yarp::conf::vocab32_t initialInteractionMode {0};
    std::atomic<yarp::dev::InteractionModeEnum> actualInteractionMode {yarp::dev::InteractionModeEnum::VOCAB_IM_UNKNOWN};

    std::mutex pidMutex;

    yarp::dev::Pid * activePid {nullptr};
    yarp::dev::Pid positionPid;
    yarp::dev::Pid impedancePid;

    double positionReference {0.0};
    double errorLimit {0.0};
    double proportionalError {0.0};
    double integralError {0.0};

    TrapezoidalTrajectory trajectory;

    std::atomic<bool> enableCsv {false};
};

} // namespace roboticslab

#endif // __TECHNOSOFT_IPOS_EXTERNAL_HPP__
