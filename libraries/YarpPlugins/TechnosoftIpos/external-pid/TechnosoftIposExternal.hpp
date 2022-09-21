// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __TECHNOSOFT_IPOS_EXTERNAL_HPP__
#define __TECHNOSOFT_IPOS_EXTERNAL_HPP__

#include <mutex>

#include <yarp/conf/numeric.h>

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

    bool synchronize() override;

    //  --------- IControlModeRaw declarations. Implementation in IControlModeRawImpl.cpp ---------

    bool getControlModeRaw(int j, int * mode) override;
    bool getControlModesRaw(int * modes) override;
    bool getControlModesRaw(int n_joint, const int * joints, int * modes) override;
    bool setControlModeRaw(int j, int mode) override;
    bool setControlModesRaw(int * modes) override;
    bool setControlModesRaw(int n_joint, const int * joints, int * modes) override;

    //  ---------- IImpedanceControlRaw declarations. Implementation in IImpedanceControlRawImpl.cpp ----------

    //bool getAxes(int * ax) override;
    bool getImpedanceRaw(int j, double * stiffness, double * damping) override;
    bool setImpedanceRaw(int j, double stiffness, double damping) override;
    bool setImpedanceOffsetRaw(int j, double offset) override;
    bool getImpedanceOffsetRaw(int j, double * offset) override;
    bool getCurrentImpedanceLimitRaw(int j, double * min_stiff, double * max_stiff, double * min_damp, double * max_damp) override;

    //  ---------- IInteractionModeRaw declarations. Implementation in IInteractionModeRawImpl.cpp ----------

    bool getInteractionModeRaw(int axis, yarp::dev::InteractionModeEnum * mode) override;
    bool getInteractionModesRaw(int n_joints, int * joints, yarp::dev::InteractionModeEnum * modes) override;
    bool getInteractionModesRaw(yarp::dev::InteractionModeEnum * modes) override;
    bool setInteractionModeRaw(int axis, yarp::dev::InteractionModeEnum mode) override;
    bool setInteractionModesRaw(int n_joints, int * joints, yarp::dev::InteractionModeEnum * modes) override;
    bool setInteractionModesRaw(yarp::dev::InteractionModeEnum * modes) override;

    //  --------- IPidControlRaw declarations. Implementation in IPidControlRawImpl.cpp ---------

    bool setPidRaw(const yarp::dev::PidControlTypeEnum & pidtype, int j, const yarp::dev::Pid & pid) override;
    bool setPidsRaw(const yarp::dev::PidControlTypeEnum & pidtype, const yarp::dev::Pid * pids) override;
    bool setPidReferenceRaw(const yarp::dev::PidControlTypeEnum & pidtype, int j, double ref) override;
    bool setPidReferencesRaw(const yarp::dev::PidControlTypeEnum & pidtype, const double * refs) override;
    bool setPidErrorLimitRaw(const yarp::dev::PidControlTypeEnum & pidtype, int j, double limit) override;
    bool setPidErrorLimitsRaw(const yarp::dev::PidControlTypeEnum & pidtype, const double * limits) override;
    bool getPidErrorRaw(const yarp::dev::PidControlTypeEnum & pidtype, int j, double * err) override;
    bool getPidErrorsRaw(const yarp::dev::PidControlTypeEnum & pidtype, double * errs) override;
    bool getPidOutputRaw(const yarp::dev::PidControlTypeEnum & pidtype, int j, double * out) override;
    bool getPidOutputsRaw(const yarp::dev::PidControlTypeEnum & pidtype, double * outs) override;
    bool getPidRaw(const yarp::dev::PidControlTypeEnum & pidtype, int j, yarp::dev::Pid * pid) override;
    bool getPidsRaw(const yarp::dev::PidControlTypeEnum & pidtype, yarp::dev::Pid * pids) override;
    bool getPidReferenceRaw(const yarp::dev::PidControlTypeEnum & pidtype, int j, double * ref) override;
    bool getPidReferencesRaw(const yarp::dev::PidControlTypeEnum & pidtype, double * refs) override;
    bool getPidErrorLimitRaw(const yarp::dev::PidControlTypeEnum & pidtype, int j, double * limit) override;
    bool getPidErrorLimitsRaw(const yarp::dev::PidControlTypeEnum & pidtype, double * limits) override;
    bool resetPidRaw(const yarp::dev::PidControlTypeEnum & pidtype, int j) override;
    bool disablePidRaw(const yarp::dev::PidControlTypeEnum & pidtype, int j) override;
    bool enablePidRaw(const yarp::dev::PidControlTypeEnum & pidtype, int j) override;
    bool setPidOffsetRaw(const yarp::dev::PidControlTypeEnum & pidtype, int j, double v) override;
    bool isPidEnabledRaw(const yarp::dev::PidControlTypeEnum & pidtype, int j, bool * enabled) override;

    // ------- IPositionControlRaw declarations. Implementation in IPositionControlRawImpl.cpp -------

    //bool getAxes(int * ax) override;
    bool positionMoveRaw(int j, double ref) override;
    bool positionMoveRaw(const double * refs) override;
    bool positionMoveRaw(int n_joint, const int * joints, const double * refs) override;
    bool relativeMoveRaw(int j, double delta) override;
    bool relativeMoveRaw(const double * deltas) override;
    bool relativeMoveRaw(int n_joint, const int * joints, const double * deltas) override;
    bool checkMotionDoneRaw(int j, bool * flag) override;
    bool checkMotionDoneRaw(bool * flag) override;
    bool checkMotionDoneRaw(int n_joint, const int * joints, bool * flag) override;
    bool setRefSpeedRaw(int j, double sp) override;
    bool setRefSpeedsRaw(const double * spds) override;
    bool setRefSpeedsRaw(int n_joint, const int * joints, const double * spds) override;
    bool setRefAccelerationRaw(int j, double acc) override;
    bool setRefAccelerationsRaw(const double * accs) override;
    bool setRefAccelerationsRaw(int n_joint, const int * joints, const double * accs) override;
    bool getRefSpeedRaw(int j, double * ref) override;
    bool getRefSpeedsRaw(double * spds) override;
    bool getRefSpeedsRaw(int n_joint, const int * joints, double * spds) override;
    bool getRefAccelerationRaw(int j, double * acc) override;
    bool getRefAccelerationsRaw(double * accs) override;
    bool getRefAccelerationsRaw(int n_joint, const int * joints, double * accs) override;
    bool stopRaw(int j) override;
    bool stopRaw() override;
    bool stopRaw(int n_joint, const int * joints) override;
    bool getTargetPositionRaw(int joint, double * ref) override;
    bool getTargetPositionsRaw(double * refs) override;
    bool getTargetPositionsRaw(int n_joint, const int * joints, double * refs) override;

    // ------- IPositionDirectRaw declarations. Implementation in IPositionDirectRawImpl.cpp -------

    //bool getAxes(int * ax) override;
    bool setPositionRaw(int j, double ref) override;
    bool setPositionsRaw(const double * refs) override;
    bool setPositionsRaw(int n_joint, const int * joints, const double * refs) override;
    bool getRefPositionRaw(int joint, double * ref) override;
    bool getRefPositionsRaw(double * refs) override;
    bool getRefPositionsRaw(int n_joint, const int *joints, double * refs) override;

    // ------- IRemoteVariablesRaw declarations. Implementation in IRemoteVariablesRawImpl.cpp -------

    bool getRemoteVariableRaw(std::string key, yarp::os::Bottle & val) override;
    bool setRemoteVariableRaw(std::string key, const yarp::os::Bottle & val) override;
    bool getRemoteVariablesListRaw(yarp::os::Bottle * listOfKeys) override;

    //  --------- IVelocityControlRaw declarations. Implementation in IVelocityControlRawImpl.cpp ---------

    //bool getAxes(int * ax) override;
    bool velocityMoveRaw(int j, double sp) override;
    bool velocityMoveRaw(const double * sp) override;
    bool velocityMoveRaw(int n_joint, const int * joints, const double * spds) override;
    bool getRefVelocityRaw(int joint, double * vel) override;
    bool getRefVelocitiesRaw(double * vels) override;
    bool getRefVelocitiesRaw(int n_joint, const int * joints, double * vels) override;
    //bool setRefAccelerationRaw(int j, double acc) override;
    //bool setRefAccelerationsRaw(const double * accs) override;
    //bool setRefAccelerationsRaw(int n_joint, const int * joints, const double * accs) override;
    //bool getRefAccelerationRaw(int j, double * acc) override;
    //bool getRefAccelerationsRaw(double * accs) override;
    //bool getRefAccelerationsRaw(int n_joint, const int * joints, double * accs) override;
    //bool stopRaw(int j) override;
    //bool stopRaw() override;
    //bool stopRaw(int n_joint, const int *joints) override;

private:
    void interpretModesOfOperation(std::int8_t modesOfOperation) override;
    void reset() override;

    yarp::conf::vocab32_t initialInteractionMode {0};

    std::atomic<double> impedanceStiffness {0.0};
    std::atomic<double> impedanceDamping {0.0};
    std::atomic<double> impedanceOffset {0.0};

    double minImpedanceStiffness {0.0};
    double maxImpedanceStiffness {0.0};
    double minImpedanceDamping {0.0};
    double maxImpedanceDamping {0.0};

    std::mutex pidMutex;
    yarp::dev::Pid positionPid;

    TrapezoidalTrajectory positionTrajectory;
    std::atomic<double> targetPosition {0.0};
};

} // namespace roboticslab

#endif // __TECHNOSOFT_IPOS_EXTERNAL_HPP__
