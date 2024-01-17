// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __FAKE_JOINT_HPP__
#define __FAKE_JOINT_HPP__

#include <string>

#include <yarp/os/SystemClock.h>
#include <yarp/dev/ControlBoardInterfaces.h>

#include "ICanBusSharer.hpp"

constexpr auto JOINT_TYPE = yarp::dev::VOCAB_JOINTTYPE_REVOLUTE; // yarpmotorgui can't handle VOCAB_JOINTTYPE_UNKNOWN

namespace roboticslab
{

/**
 * @ingroup YarpPlugins
 * @defgroup FakeJoint
 * @brief Contains roboticslab::FakeJoint.
 */

/**
 * @ingroup FakeJoint
 * @brief Implementation for a fake joint (instant movement) as a single CAN bus joint (control board raw interfaces).
 */
class FakeJoint : public yarp::dev::DeviceDriver,
                  public yarp::dev::IAmplifierControlRaw,
                  public yarp::dev::IAxisInfoRaw,
                  public yarp::dev::IControlCalibrationRaw,
                  public yarp::dev::IControlLimitsRaw,
                  public yarp::dev::IControlModeRaw,
                  public yarp::dev::ICurrentControlRaw,
                  public yarp::dev::IEncodersTimedRaw,
                  public yarp::dev::IImpedanceControlRaw,
                  public yarp::dev::IInteractionModeRaw,
                  public yarp::dev::IMotorRaw,
                  public yarp::dev::IMotorEncodersRaw,
                  public yarp::dev::IPidControlRaw,
                  public yarp::dev::IPositionControlRaw,
                  public yarp::dev::IPositionDirectRaw,
                  public yarp::dev::IPWMControlRaw,
                  public yarp::dev::IRemoteVariablesRaw,
                  public yarp::dev::ITorqueControlRaw,
                  public yarp::dev::IVelocityControlRaw,
                  public ICanBusSharer
{
public:
    //  --------- DeviceDriver declarations. Implementation in DeviceDriverImpl.cpp ---------

    bool open(yarp::os::Searchable & config) override;

    bool close() override
    { return true; }

    //  --------- ICanBusSharer declarations ---------

    unsigned int getId() override
    { return 0; }
    bool notifyMessage(const can_message & message) override
    { return true; }
    bool initialize() override
    { return true; }
    bool finalize() override
    { return true; }
    bool registerSender(ICanSenderDelegate * sender) override
    { return true; }
    bool synchronize(double timestamp) override
    { return true; }

    //  --------- IAmplifierControlRaw declarations ---------

    bool enableAmpRaw(int j) override
    { return true; }
    bool disableAmpRaw(int j) override
    { return true; }
    bool getAmpStatusRaw(int j, int * v) override
    { *v = 0; return true; }
    bool getAmpStatusRaw(int * st) override
    { *st = 0; return true; }
    //bool getCurrentRaw(int j, double * val) override;
    //bool getCurrentsRaw(double * vals) override;
    bool getMaxCurrentRaw(int j, double * v) override
    { *v = 0.0; return true; }
    bool setMaxCurrentRaw(int j, double v) override
    { return true; }
    bool getNominalCurrentRaw(int m, double * val) override
    { *val = 0.0; return true; }
    bool setNominalCurrentRaw(int m, double val) override
    { return true; }
    bool getPeakCurrentRaw(int m, double * val) override
    { *val = 0.0; return true; }
    bool setPeakCurrentRaw(int m, double val) override
    { return true; }
    bool getPWMRaw(int j, double * val) override
    { *val = 0.0; return true; }
    bool getPWMLimitRaw(int j, double * val) override
    { *val = 0.0; return true; }
    bool setPWMLimitRaw(int j, double val) override
    { return true; }
    bool getPowerSupplyVoltageRaw(int j, double * val) override
    { *val = 0.0; return true; }

    //  --------- IAxisInfoRaw declarations ---------

    bool getAxisNameRaw(int axis, std::string & name) override
    {  name = jointName; return true; }
    bool getJointTypeRaw(int axis, yarp::dev::JointTypeEnum & type) override
    { type = JOINT_TYPE; return true; }

    //  --------- IControlCalibrationRaw declarations ---------

    bool calibrateAxisWithParamsRaw(int axis, unsigned int type, double p1, double p2, double p3) override
    { return true; }
    bool setCalibrationParametersRaw(int axis, const yarp::dev::CalibrationParameters & params) override
    { return true; }
    bool calibrationDoneRaw(int j) override
    { return true; }

    //  --------- IControlLimitsRaw declarations ---------

    bool setLimitsRaw(int axis, double min, double max) override
    { return true; }
    bool getLimitsRaw(int axis, double * min, double * max) override
    { *min = *max = 0.0; return true; }
    bool setVelLimitsRaw(int axis, double min, double max) override
    { return true; }
    bool getVelLimitsRaw(int axis, double * min, double * max) override
    { *min = *max = 0.0; return true; }

    //  --------- IControlModeRaw declarations ---------

    bool getControlModeRaw(int j, int * mode) override
    { *mode = controlMode; return true; }
    bool getControlModesRaw(int * modes) override
    { return getControlModeRaw(0, &modes[0]); }
    bool getControlModesRaw(int n_joint, const int * joints, int * modes) override
    { return getControlModeRaw(0, &modes[0]); }
    bool setControlModeRaw(int j, int mode) override
    { controlMode = mode; return true; }
    bool setControlModesRaw(int * modes) override
    { return setControlModeRaw(0, modes[0]); }
    bool setControlModesRaw(int n_joint, const int * joints, int * modes) override
    { return setControlModeRaw(0, modes[0]); }

    //  --------- ICurrentControlRaw declarations ---------

    //bool getNumberOfMotorsRaw(int * ax) override;
    bool getCurrentRaw(int m, double * curr) override
    { *curr = 0.0; return true; }
    bool getCurrentsRaw(double * currs) override
    { return getCurrentRaw(0, &currs[0]); }
    bool getCurrentRangeRaw(int m, double * min, double * max) override
    { *min = *max = 0.0; return true; }
    bool getCurrentRangesRaw(double * mins, double * maxs) override
    { return getCurrentRangeRaw(0, &mins[0], &maxs[0]); }
    bool setRefCurrentRaw(int m, double curr) override
    { return true; }
    bool setRefCurrentsRaw(const double * currs) override
    { return true; }
    bool setRefCurrentsRaw(int n_motor, const int * motors, const double * currs) override
    { return true; }
    bool getRefCurrentRaw(int m, double * curr) override
    { *curr = 0.0; return true; }
    bool getRefCurrentsRaw(double * currs) override
    { return getRefCurrentRaw(0, &currs[0]); }

    //  ---------- IEncodersRaw declarations ----------.

    bool getAxes(int * ax) override
    { *ax = 1; return true; }
    bool resetEncoderRaw(int j) override
    { return true; }
    bool resetEncodersRaw() override
    { return true; }
    bool setEncoderRaw(int j, double val) override
    { return true; }
    bool setEncodersRaw(const double * vals) override
    { return true; }
    bool getEncoderRaw(int j, double * v) override
    { *v = 0.0; return true; }
    bool getEncodersRaw(double * encs) override
    { return getEncoderRaw(0, &encs[0]); }
    bool getEncoderSpeedRaw(int j, double * spd) override
    { *spd = 0.0; return true; }
    bool getEncoderSpeedsRaw(double * spds) override
    { return getEncoderRaw(0, &spds[0]); }
    bool getEncoderAccelerationRaw(int j, double * acc) override
    { *acc = 0.0; return true; }
    bool getEncoderAccelerationsRaw(double * accs) override
    { return getEncoderAccelerationRaw(0, &accs[0]); }

    //  ---------- IEncodersTimedRaw declarations ----------

    bool getEncoderTimedRaw(int j, double * enc, double * time) override
    { *enc = 0.0; *time = yarp::os::SystemClock::nowSystem(); return true; }
    bool getEncodersTimedRaw(double * encs, double * times) override
    { return getEncoderTimedRaw(0, &encs[0], &times[0]); }

    //  --------- IImpedanceControlRaw declarations ---------

    //bool getAxes(int * ax) override;
    bool getImpedanceRaw(int j, double * stiffness, double * damping) override
    { *stiffness = *damping = 0.0; return true; }
    bool setImpedanceRaw(int j, double stiffness, double damping) override
    { return true; }
    bool setImpedanceOffsetRaw(int j, double offset) override
    { return true; }
    bool getImpedanceOffsetRaw(int j, double * offset) override
    { *offset = 0.0; return true; }
    bool getCurrentImpedanceLimitRaw(int j, double * min_stiff, double * max_stiff, double * min_damp, double * max_damp) override
    { *min_stiff = *max_stiff = *min_damp = *max_damp = 0.0; return true; }

    // ------- IInteractionModeRaw declarations -------

    bool getInteractionModeRaw(int axis, yarp::dev::InteractionModeEnum * mode) override
    { *mode = interactionMode; return true; }
    bool getInteractionModesRaw(yarp::dev::InteractionModeEnum * modes) override
    { return getInteractionModeRaw(0, &modes[0]); }
    bool getInteractionModesRaw(int n_joints, int * joints, yarp::dev::InteractionModeEnum * modes) override
    { return getInteractionModeRaw(0, &modes[0]); }
    bool setInteractionModeRaw(int axis, yarp::dev::InteractionModeEnum mode) override
    { interactionMode = mode; return true; }
    bool setInteractionModesRaw(yarp::dev::InteractionModeEnum * modes) override
    { return true; }
    bool setInteractionModesRaw(int n_joints, int * joints, yarp::dev::InteractionModeEnum * modes) override
    { return true; }

    //  --------- IMotorRaw declarations ---------

    bool getNumberOfMotorsRaw(int * num) override
    { return getAxes(num); }
    bool getTemperatureRaw(int m, double * val) override
    { *val = 0.0; return true; }
    bool getTemperaturesRaw(double * vals) override
    { return getTemperatureRaw(0, &vals[0]); }
    bool getTemperatureLimitRaw(int m, double * temp) override
    { *temp = 0.0; return true; }
    bool setTemperatureLimitRaw(int m, double temp) override
    { return true; }
    bool getGearboxRatioRaw(int m, double * val) override
    { *val = 0.0; return true; }
    bool setGearboxRatioRaw(int m, double val) override
    { return true; }

    //  --------- IMotorEncodersRaw declarations ---------

    bool getNumberOfMotorEncodersRaw(int * num) override
    { return getAxes(num); }
    bool resetMotorEncoderRaw(int m) override
    { return true; }
    bool resetMotorEncodersRaw() override
    { return true; }
    bool setMotorEncoderCountsPerRevolutionRaw(int m, double cpr) override
    { return true; }
    bool getMotorEncoderCountsPerRevolutionRaw(int m, double * cpr) override
    { *cpr = 0.0; return true; }
    bool setMotorEncoderRaw(int m, double val) override
    { return true; }
    bool setMotorEncodersRaw(const double * vals) override
    { return true; }
    bool getMotorEncoderRaw(int m, double * v) override
    { *v = 0.0; return true; }
    bool getMotorEncodersRaw(double * encs) override
    { return getMotorEncoderRaw(0, &encs[0]); }
    bool getMotorEncoderTimedRaw(int m, double * enc, double * stamp) override
    { *enc = 0.0; *stamp = yarp::os::SystemClock::nowSystem(); return true; }
    bool getMotorEncodersTimedRaw(double * encs, double * stamps) override
    { return getMotorEncoderTimedRaw(0, &encs[0], &stamps[0]); }
    bool getMotorEncoderSpeedRaw(int m, double * sp) override
    { *sp = 0.0; return true; }
    bool getMotorEncoderSpeedsRaw(double *spds) override
    { return getMotorEncoderSpeedRaw(0, &spds[0]); }
    bool getMotorEncoderAccelerationRaw(int m, double * acc) override
    { *acc = 0.0; return true; }
    bool getMotorEncoderAccelerationsRaw(double * accs) override
    { return getMotorEncoderAccelerationRaw(0, &accs[0]); }

    //  --------- IPidControlRaw declarations ---------

    bool setPidRaw(const yarp::dev::PidControlTypeEnum & pidtype, int j, const yarp::dev::Pid & pid) override
    { return true; }
    bool setPidsRaw(const yarp::dev::PidControlTypeEnum & pidtype, const yarp::dev::Pid * pids) override
    { return true; }
    bool setPidReferenceRaw(const yarp::dev::PidControlTypeEnum & pidtype, int j, double ref) override
    { return true; }
    bool setPidReferencesRaw(const yarp::dev::PidControlTypeEnum & pidtype, const double * refs) override
    { return true; }
    bool setPidErrorLimitRaw(const yarp::dev::PidControlTypeEnum & pidtype, int j, double limit) override
    { return true; }
    bool setPidErrorLimitsRaw(const yarp::dev::PidControlTypeEnum & pidtype, const double * limits) override
    { return true; }
    bool getPidErrorRaw(const yarp::dev::PidControlTypeEnum & pidtype, int j, double * err) override
    { *err = 0.0; return true; }
    bool getPidErrorsRaw(const yarp::dev::PidControlTypeEnum & pidtype, double * errs) override
    { return getPidErrorRaw(pidtype, 0, &errs[0]); }
    bool getPidOutputRaw(const yarp::dev::PidControlTypeEnum & pidtype, int j, double * out) override
    { *out = 0.0; return true; }
    bool getPidOutputsRaw(const yarp::dev::PidControlTypeEnum & pidtype, double * outs) override
    { return getPidOutputRaw(pidtype, 0, &outs[0]); }
    bool getPidRaw(const yarp::dev::PidControlTypeEnum & pidtype, int j, yarp::dev::Pid * pid) override
    { return true; }
    bool getPidsRaw(const yarp::dev::PidControlTypeEnum & pidtype, yarp::dev::Pid * pids) override
    { return true; }
    bool getPidReferenceRaw(const yarp::dev::PidControlTypeEnum & pidtype, int j, double * ref) override
    { *ref = 0.0; return true; }
    bool getPidReferencesRaw(const yarp::dev::PidControlTypeEnum & pidtype, double * refs) override
    { return getPidReferenceRaw(pidtype, 0, &refs[0]); }
    bool getPidErrorLimitRaw(const yarp::dev::PidControlTypeEnum & pidtype, int j, double * limit) override
    { *limit = 0.0; return true; }
    bool getPidErrorLimitsRaw(const yarp::dev::PidControlTypeEnum & pidtype, double * limits) override
    { return getPidErrorLimitRaw(pidtype, 0, &limits[0]); }
    bool resetPidRaw(const yarp::dev::PidControlTypeEnum & pidtype, int j) override
    { return true; }
    bool disablePidRaw(const yarp::dev::PidControlTypeEnum & pidtype, int j) override
    { return true; }
    bool enablePidRaw(const yarp::dev::PidControlTypeEnum & pidtype, int j) override
    { return true; }
    bool setPidOffsetRaw(const yarp::dev::PidControlTypeEnum & pidtype, int j, double v) override
    { return true; }
    bool isPidEnabledRaw(const yarp::dev::PidControlTypeEnum & pidtype, int j, bool * enabled) override
    { *enabled = true; return true; }

    // ------- IPositionControlRaw declarations -------

    //bool getAxesRaw(int * ax) override;
    bool positionMoveRaw(int j, double ref) override
    { return true; }
    bool positionMoveRaw(const double * refs) override
    { return true; }
    bool positionMoveRaw(int n_joint, const int * joints, const double * refs) override
    { return true; }
    bool relativeMoveRaw(int j, double delta) override
    { return true; }
    bool relativeMoveRaw(const double * deltas) override
    { return true; }
    bool relativeMoveRaw(int n_joint, const int * joints, const double * deltas) override
    { return true; }
    bool checkMotionDoneRaw(int j, bool * flag) override
    { *flag = true; return true; }
    bool checkMotionDoneRaw(bool * flag) override
    { return checkMotionDoneRaw(0, flag); }
    bool checkMotionDoneRaw(int n_joint, const int * joints, bool * flag) override
    { return checkMotionDoneRaw(0, flag); }
    bool setRefSpeedRaw(int j, double sp) override
    { return true; }
    bool setRefSpeedsRaw(const double * spds) override
    { return true; }
    bool setRefSpeedsRaw(int n_joint, const int * joints, const double * spds) override
    { return true; }
    bool setRefAccelerationRaw(int j, double acc) override
    { return true; }
    bool setRefAccelerationsRaw(const double * accs) override
    { return true; }
    bool setRefAccelerationsRaw(int n_joint, const int * joints, const double * accs) override
    { return true; }
    bool getRefSpeedRaw(int j, double * spd) override
    { *spd = 0.0; return true; }
    bool getRefSpeedsRaw(double * spds) override
    { return getRefSpeedRaw(0, &spds[0]); }
    bool getRefSpeedsRaw(int n_joint, const int * joints, double * spds) override
    { return getRefSpeedRaw(0, &spds[0]); }
    bool getRefAccelerationRaw(int j, double * acc) override
    { *acc = 0.0; return true; }
    bool getRefAccelerationsRaw(double * accs) override
    { return getRefAccelerationRaw(0, &accs[0]); }
    bool getRefAccelerationsRaw(int n_joint, const int * joints, double * accs) override
    { return getRefAccelerationRaw(0, &accs[0]); }
    bool stopRaw(int j) override
    { return true; }
    bool stopRaw() override
    { return true; }
    bool stopRaw(int n_joint, const int *joints) override
    { return true; }
    bool getTargetPositionRaw(int joint, double * ref) override
    { *ref = 0.0; return true; }
    bool getTargetPositionsRaw(double * refs) override
    { return getTargetPositionRaw(0, &refs[0]); }
    bool getTargetPositionsRaw(int n_joint, const int * joints, double * refs) override
    { return getTargetPositionRaw(0, &refs[0]); }

    // ------- IPositionDirectRaw declarations -------

    //bool getAxesRaw(int * ax) override;
    bool setPositionRaw(int j, double ref) override
    { return true; }
    bool setPositionsRaw(const double * refs) override
    { return true; }
    bool setPositionsRaw(int n_joint, const int * joints, const double * refs) override
    { return true; }
    bool getRefPositionRaw(int joint, double * ref) override
    { *ref = 0.0; return true; }
    bool getRefPositionsRaw(double * refs) override
    { return getRefPositionRaw(0, &refs[0]); }
    bool getRefPositionsRaw(int n_joint, const int * joints, double * refs) override
    { return getRefPositionRaw(0, &refs[0]); }

    //  --------- IPWMControl declarations ---------

    //bool getNumberOfMotorsRaw(int * number) override;
    bool setRefDutyCycleRaw(int m, double ref) override
    { return true; }
    bool setRefDutyCyclesRaw(const double * refs) override
    { return true; }
    bool getRefDutyCycleRaw(int m, double * ref) override
    { *ref = 0.0; return true; }
    bool getRefDutyCyclesRaw(double * refs) override
    { return getRefDutyCycleRaw(0, &refs[0]); }
    bool getDutyCycleRaw(int m, double * val) override
    { *val = 0.0; return true; }
    bool getDutyCyclesRaw(double * vals) override
    { return getDutyCycleRaw(0, &vals[0]); }

    // ------- IRemoteVariablesRaw declarations -------

    bool getRemoteVariableRaw(std::string key, yarp::os::Bottle & val) override
    { return true; }
    bool setRemoteVariableRaw(std::string key, const yarp::os::Bottle & val) override
    { return true; }
    bool getRemoteVariablesListRaw(yarp::os::Bottle * listOfKeys) override
    { return true; }

    // -------- ITorqueControlRaw declarations --------

    //bool getAxesRaw(int * ax) override;
    bool getRefTorqueRaw(int j, double * t) override
    { *t = 0.0; return true; }
    bool getRefTorquesRaw(double * t) override
    { return getRefTorqueRaw(0, &t[0]); }
    bool setRefTorqueRaw(int j, double t) override
    { return true; }
    bool setRefTorquesRaw(const double * t) override
    { return true; }
    bool setRefTorquesRaw(int n_joint, const int * joints, const double * t) override
    { return true; }
    bool getMotorTorqueParamsRaw(int j, yarp::dev::MotorTorqueParameters * params) override
    { return true; }
    bool setMotorTorqueParamsRaw(int j, const yarp::dev::MotorTorqueParameters params) override
    { return true; }
    bool getTorqueRaw(int j, double * t) override
    { *t = 0.0; return true; }
    bool getTorquesRaw(double * t) override
    { return getTorqueRaw(0, &t[0]); }
    bool getTorqueRangeRaw(int j, double * min, double * max) override
    { *min = *max = 0.0; return true; }
    bool getTorqueRangesRaw(double * mins, double * maxs) override
    { return getTorqueRangeRaw(0, &mins[0], &maxs[0]); }

    //  --------- IVelocityControl declarations ---------

    //bool getAxesRaw(int * ax) override;
    bool velocityMoveRaw(int j, double spd) override
    { return true; }
    bool velocityMoveRaw(const double * spds) override
    { return true; }
    bool velocityMoveRaw(int n_joint, const int * joints, const double * spds) override
    { return true; }
    bool getRefVelocityRaw(int joint, double * vel) override
    { *vel = 0.0; return true; }
    bool getRefVelocitiesRaw(double * vels) override
    { return getRefVelocityRaw(0, &vels[0]); }
    bool getRefVelocitiesRaw(int n_joint, const int * joints, double * vels) override
    { return getRefVelocityRaw(0, &vels[0]); }
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
    std::string jointName;
    int controlMode {VOCAB_CM_CONFIGURED};
    yarp::dev::InteractionModeEnum interactionMode {yarp::dev::VOCAB_IM_UNKNOWN};
};

} // namespace roboticslab

#endif // __FAKE_JOINT_HPP__
