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
    //  --------- DeviceDriver declarations. Implementation in DeviceDriverImpl.cpp ---------

    bool open(yarp::os::Searchable & config) override;
    bool close() override;

    //  --------- ICanBusSharer declarations. Implementation in ICanBusSharerImpl.cpp ---------

    bool initialize() override;
    bool synchronize(double timestamp) override;

    //  --------- IControlModeRaw declarations. Implementation in IControlModeRawImpl.cpp ---------

    bool getControlModeRaw(int j, int * mode) override;
    bool setControlModeRaw(int j, int mode) override;

    //  ---------- IImpedanceControlRaw declarations. Implementation in IImpedanceControlRawImpl.cpp ----------

    bool getImpedanceRaw(int j, double * stiffness, double * damping) override;
    bool setImpedanceRaw(int j, double stiffness, double damping) override;
    bool setImpedanceOffsetRaw(int j, double offset) override;
    bool getImpedanceOffsetRaw(int j, double * offset) override;
    bool getCurrentImpedanceLimitRaw(int j, double * min_stiff, double * max_stiff, double * min_damp, double * max_damp) override;

    //  ---------- IInteractionModeRaw declarations. Implementation in IInteractionModeRawImpl.cpp ----------

    bool getInteractionModeRaw(int axis, yarp::dev::InteractionModeEnum * mode) override;
    bool setInteractionModeRaw(int axis, yarp::dev::InteractionModeEnum mode) override;

    //  --------- IPidControlRaw declarations. Implementation in IPidControlRawImpl.cpp ---------

    bool setPidRaw(const yarp::dev::PidControlTypeEnum & pidtype, int j, const yarp::dev::Pid & pid) override;
    bool setPidReferenceRaw(const yarp::dev::PidControlTypeEnum & pidtype, int j, double ref) override;
    bool setPidErrorLimitRaw(const yarp::dev::PidControlTypeEnum & pidtype, int j, double limit) override;
    bool getPidErrorRaw(const yarp::dev::PidControlTypeEnum & pidtype, int j, double * err) override;
    bool getPidOutputRaw(const yarp::dev::PidControlTypeEnum & pidtype, int j, double * out) override;
    bool getPidRaw(const yarp::dev::PidControlTypeEnum & pidtype, int j, yarp::dev::Pid * pid) override;
    bool getPidReferenceRaw(const yarp::dev::PidControlTypeEnum & pidtype, int j, double * ref) override;
    bool getPidErrorLimitRaw(const yarp::dev::PidControlTypeEnum & pidtype, int j, double * limit) override;
    bool resetPidRaw(const yarp::dev::PidControlTypeEnum & pidtype, int j) override;
    bool disablePidRaw(const yarp::dev::PidControlTypeEnum & pidtype, int j) override;
    bool enablePidRaw(const yarp::dev::PidControlTypeEnum & pidtype, int j) override;
    bool setPidOffsetRaw(const yarp::dev::PidControlTypeEnum & pidtype, int j, double v) override;
    bool isPidEnabledRaw(const yarp::dev::PidControlTypeEnum & pidtype, int j, bool * enabled) override;

    // ------- IPositionControlRaw declarations. Implementation in IPositionControlRawImpl.cpp -------

    bool positionMoveRaw(int j, double ref) override;
    bool relativeMoveRaw(int j, double delta) override;
    bool checkMotionDoneRaw(int j, bool * flag) override;
    bool setRefSpeedRaw(int j, double sp) override;
    bool setRefAccelerationRaw(int j, double acc) override;
    bool getRefSpeedRaw(int j, double * ref) override;
    bool getRefAccelerationRaw(int j, double * acc) override;
    bool stopRaw(int j) override;
    bool getTargetPositionRaw(int joint, double * ref) override;

    // ------- IPositionDirectRaw declarations. Implementation in IPositionDirectRawImpl.cpp -------

    bool setPositionRaw(int j, double ref) override;
    bool getRefPositionRaw(int joint, double * ref) override;

    // ------- IRemoteVariablesRaw declarations. Implementation in IRemoteVariablesRawImpl.cpp -------

    bool getRemoteVariableRaw(std::string key, yarp::os::Bottle & val) override;
    bool setRemoteVariableRaw(std::string key, const yarp::os::Bottle & val) override;
    bool getRemoteVariablesListRaw(yarp::os::Bottle * listOfKeys) override;

    //  --------- IVelocityControlRaw declarations. Implementation in IVelocityControlRawImpl.cpp ---------

    bool velocityMoveRaw(int j, double sp) override;
    bool getRefVelocityRaw(int joint, double * vel) override;

private:
    void interpretModesOfOperation(std::int8_t modesOfOperation) override;
    void onPositionLimitTriggered() override;
    void reset() override;

    yarp::conf::vocab32_t initialInteractionMode {0};
    std::atomic<yarp::dev::InteractionModeEnum> actualInteractionMode {yarp::dev::InteractionModeEnum::VOCAB_IM_UNKNOWN};

    double minStiffness {0.0};
    double maxStiffness {0.0};
    double minDamping {0.0};
    double maxDamping {0.0};

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
