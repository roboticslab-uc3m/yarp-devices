// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __FAKE_JOINT_HPP__
#define __FAKE_JOINT_HPP__

#include <string>

#include <yarp/conf/version.h>

#include <yarp/os/SystemClock.h>
#include <yarp/dev/ControlBoardInterfaces.h>

#include "ICanBusSharer.hpp"
#include "FakeJoint_ParamsParser.h"

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
constexpr auto JOINT_TYPE = yarp::dev::JointTypeEnum::VOCAB_JOINTTYPE_REVOLUTE; // yarpmotorgui can't handle VOCAB_JOINTTYPE_UNKNOWN
#else
constexpr auto JOINT_TYPE = yarp::dev::VOCAB_JOINTTYPE_REVOLUTE; // yarpmotorgui can't handle VOCAB_JOINTTYPE_UNKNOWN
#endif

/**
 * @ingroup YarpPlugins
 * @defgroup FakeJoint
 * @brief Contains FakeJoint.
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
                  public roboticslab::ICanBusSharer,
                  public FakeJoint_ParamsParser
{
public:
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    using return_t = yarp::dev::ReturnValue;
    constexpr static auto ret_ok = yarp::dev::ReturnValue::return_code::return_value_ok;
#else
    using return_t = bool;
    constexpr static auto ret_ok = true;
#endif

    //  --------- DeviceDriver declarations. Implementation in DeviceDriverImpl.cpp ---------

    bool open(yarp::os::Searchable & config) override;

    bool close() override
    { return true; }

    //  --------- ICanBusSharer declarations ---------

    unsigned int getId() override
    { return 0; }
    bool notifyMessage(const roboticslab::can_message & message) override
    { return true; }
    bool initialize() override
    { return true; }
    bool finalize() override
    { return true; }
    bool registerSender(roboticslab::ICanSenderDelegate * sender) override
    { return true; }
    bool synchronize(double timestamp) override
    { return true; }

    //  --------- IAmplifierControlRaw declarations ---------

    return_t enableAmpRaw(int j) override
    { return ret_ok; }
    return_t disableAmpRaw(int j) override
    { return ret_ok; }
    return_t getAmpStatusRaw(int j, int * v) override
    { *v = 0; return ret_ok; }
    return_t getAmpStatusRaw(int * st) override
    { *st = 0; return ret_ok; }
    return_t getMaxCurrentRaw(int j, double * v) override
    { *v = 0.0; return ret_ok; }
    return_t setMaxCurrentRaw(int j, double v) override
    { return ret_ok; }
    return_t getNominalCurrentRaw(int m, double * val) override
    { *val = 0.0; return ret_ok; }
    return_t setNominalCurrentRaw(int m, double val) override
    { return ret_ok; }
    return_t getPeakCurrentRaw(int m, double * val) override
    { *val = 0.0; return ret_ok; }
    return_t setPeakCurrentRaw(int m, double val) override
    { return ret_ok; }
    return_t getPWMRaw(int j, double * val) override
    { *val = 0.0; return ret_ok; }
    return_t getPWMLimitRaw(int j, double * val) override
    { *val = 0.0; return ret_ok; }
    return_t setPWMLimitRaw(int j, double val) override
    { return ret_ok; }
    return_t getPowerSupplyVoltageRaw(int j, double * val) override
    { *val = 0.0; return ret_ok; }

    //  --------- IAxisInfoRaw declarations ---------

    return_t getAxisNameRaw(int axis, std::string & name) override
    {  name = m_jointName; return ret_ok; }
    return_t getJointTypeRaw(int axis, yarp::dev::JointTypeEnum & type) override
    { type = JOINT_TYPE; return ret_ok; }

    //  --------- IControlCalibrationRaw declarations ---------

    return_t calibrateAxisWithParamsRaw(int axis, unsigned int type, double p1, double p2, double p3) override
    { return ret_ok; }
    return_t setCalibrationParametersRaw(int axis, const yarp::dev::CalibrationParameters & params) override
    { return ret_ok; }
    return_t calibrationDoneRaw(int j) override
    { return ret_ok; }

    //  --------- IControlLimitsRaw declarations ---------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return_t setPosLimitsRaw(int axis, double min, double max) override
    { return ret_ok; }
    return_t getPosLimitsRaw(int axis, double * min, double * max) override
    { *min = *max = 0.0; return ret_ok; }
#else
    return_t setLimitsRaw(int axis, double min, double max) override
    { return ret_ok; }
    return_t getLimitsRaw(int axis, double * min, double * max) override
    { *min = *max = 0.0; return ret_ok; }
#endif
    return_t setVelLimitsRaw(int axis, double min, double max) override
    { return ret_ok; }
    return_t getVelLimitsRaw(int axis, double * min, double * max) override
    { *min = *max = 0.0; return ret_ok; }

    //  --------- IControlModeRaw declarations ---------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return_t getAvailableControlModesRaw(int j, std::vector<yarp::dev::SelectableControlModeEnum> & avail) override
    { using t = yarp::dev::SelectableControlModeEnum;
      avail = {t::VOCAB_CM_IDLE, t::VOCAB_CM_TORQUE, t::VOCAB_CM_POSITION, t::VOCAB_CM_POSITION_DIRECT, t::VOCAB_CM_VELOCITY,
               t::VOCAB_CM_VELOCITY_DIRECT, t::VOCAB_CM_CURRENT, t::VOCAB_CM_PWM, t::VOCAB_CM_MIXED, t::VOCAB_CM_FORCE_IDLE};
      return ret_ok; }
    return_t getControlModeRaw(int j, yarp::dev::ControlModeEnum & mode) override
    { mode = controlMode; return ret_ok; }
    return_t getControlModesRaw(std::vector<yarp::dev::ControlModeEnum> & modes) override
    { return getControlModeRaw(0, modes[0]); }
    return_t getControlModesRaw(const std::vector<int> & joints, std::vector<yarp::dev::ControlModeEnum> & modes) override
    { return getControlModeRaw(0, modes[0]); }
    return_t setControlModeRaw(int j, yarp::dev::SelectableControlModeEnum mode) override
    { controlMode = static_cast<yarp::dev::ControlModeEnum>(mode); return ret_ok; }
    return_t setControlModesRaw(const std::vector<yarp::dev::SelectableControlModeEnum> & modes) override
    { return setControlModeRaw(0, modes[0]); }
    return_t setControlModesRaw(const std::vector<int> & joints, const std::vector<yarp::dev::SelectableControlModeEnum> & modes) override
    { return setControlModeRaw(0, modes[0]); }
#else
    return_t getControlModeRaw(int j, int * mode) override
    { *mode = controlMode; return ret_ok; }
    return_t getControlModesRaw(int * modes) override
    { return getControlModeRaw(0, &modes[0]); }
    return_t getControlModesRaw(int n_joint, const int * joints, int * modes) override
    { return getControlModeRaw(0, &modes[0]); }
    return_t setControlModeRaw(int j, int mode) override
    { controlMode = mode; return ret_ok; }
    return_t setControlModesRaw(int * modes) override
    { return setControlModeRaw(0, modes[0]); }
    return_t setControlModesRaw(int n_joint, const int * joints, int * modes) override
    { return setControlModeRaw(0, modes[0]); }
#endif

    //  --------- ICurrentControlRaw declarations ---------

    return_t getCurrentRaw(int m, double * curr) override
    { *curr = 0.0; return ret_ok; }
    return_t getCurrentsRaw(double * currs) override
    { return getCurrentRaw(0, &currs[0]); }
    return_t getCurrentRangeRaw(int m, double * min, double * max) override
    { *min = *max = 0.0; return ret_ok; }
    return_t getCurrentRangesRaw(double * mins, double * maxs) override
    { return getCurrentRangeRaw(0, &mins[0], &maxs[0]); }
    return_t setRefCurrentRaw(int m, double curr) override
    { return ret_ok; }
    return_t setRefCurrentsRaw(const double * currs) override
    { return ret_ok; }
    return_t setRefCurrentsRaw(int n_motor, const int * motors, const double * currs) override
    { return ret_ok; }
    return_t getRefCurrentRaw(int m, double * curr) override
    { *curr = 0.0; return ret_ok; }
    return_t getRefCurrentsRaw(double * currs) override
    { return getRefCurrentRaw(0, &currs[0]); }

    //  ---------- IEncodersRaw declarations ----------.

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return_t getAxes(std::size_t & ax) override
    { ax = 1; return ret_ok; }
#else
    return_t getAxes(int * ax) override
    { *ax = 1; return ret_ok; }
#endif
    return_t resetEncoderRaw(int j) override
    { return ret_ok; }
    return_t resetEncodersRaw() override
    { return ret_ok; }
    return_t setEncoderRaw(int j, double val) override
    { return ret_ok; }
    return_t setEncodersRaw(const double * vals) override
    { return ret_ok; }
    return_t getEncoderRaw(int j, double * v) override
    { *v = 0.0; return ret_ok; }
    return_t getEncodersRaw(double * encs) override
    { return getEncoderRaw(0, &encs[0]); }
    return_t getEncoderSpeedRaw(int j, double * spd) override
    { *spd = 0.0; return ret_ok; }
    return_t getEncoderSpeedsRaw(double * spds) override
    { return getEncoderRaw(0, &spds[0]); }
    return_t getEncoderAccelerationRaw(int j, double * acc) override
    { *acc = 0.0; return ret_ok; }
    return_t getEncoderAccelerationsRaw(double * accs) override
    { return getEncoderAccelerationRaw(0, &accs[0]); }

    //  ---------- IEncodersTimedRaw declarations ----------

    return_t getEncoderTimedRaw(int j, double * enc, double * time) override
    { *enc = 0.0; *time = yarp::os::SystemClock::nowSystem(); return ret_ok; }
    return_t getEncodersTimedRaw(double * encs, double * times) override
    { return getEncoderTimedRaw(0, &encs[0], &times[0]); }

    //  --------- IImpedanceControlRaw declarations ---------

    return_t getImpedanceRaw(int j, double * stiffness, double * damping) override
    { *stiffness = *damping = 0.0; return ret_ok; }
    return_t setImpedanceRaw(int j, double stiffness, double damping) override
    { return ret_ok; }
    return_t setImpedanceOffsetRaw(int j, double offset) override
    { return ret_ok; }
    return_t getImpedanceOffsetRaw(int j, double * offset) override
    { *offset = 0.0; return ret_ok; }
    return_t getCurrentImpedanceLimitRaw(int j, double * min_stiff, double * max_stiff, double * min_damp, double * max_damp) override
    { *min_stiff = *max_stiff = *min_damp = *max_damp = 0.0; return ret_ok; }

    // ------- IInteractionModeRaw declarations -------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return_t getInteractionModeRaw(int axis, yarp::dev::InteractionModeEnum & mode) override
    { mode = interactionMode; return ret_ok; }
    return_t getInteractionModesRaw(std::vector<yarp::dev::InteractionModeEnum> & modes) override
    { return getInteractionModeRaw(0, modes[0]); }
    return_t getInteractionModesRaw(const std::vector<int> & joints, std::vector<yarp::dev::InteractionModeEnum> & modes) override
    { return getInteractionModeRaw(0, modes[0]); }
    return_t setInteractionModeRaw(int axis, yarp::dev::InteractionModeEnum mode) override
    { interactionMode = mode; return ret_ok; }
    return_t setInteractionModesRaw(const std::vector<yarp::dev::InteractionModeEnum> & modes) override
    { return ret_ok; }
    return_t setInteractionModesRaw(const std::vector<int> & joints, const std::vector<yarp::dev::InteractionModeEnum> & modes) override
    { return ret_ok; }
#else
    return_t getInteractionModeRaw(int axis, yarp::dev::InteractionModeEnum * mode) override
    { *mode = interactionMode; return ret_ok; }
    return_t getInteractionModesRaw(yarp::dev::InteractionModeEnum * modes) override
    { return getInteractionModeRaw(0, &modes[0]); }
    return_t getInteractionModesRaw(int n_joints, int * joints, yarp::dev::InteractionModeEnum * modes) override
    { return getInteractionModeRaw(0, &modes[0]); }
    return_t setInteractionModeRaw(int axis, yarp::dev::InteractionModeEnum mode) override
    { interactionMode = mode; return ret_ok; }
    return_t setInteractionModesRaw(yarp::dev::InteractionModeEnum * modes) override
    { return ret_ok; }
    return_t setInteractionModesRaw(int n_joints, int * joints, yarp::dev::InteractionModeEnum * modes) override
    { return ret_ok; }
#endif

    //  --------- IMotorRaw declarations ---------

    return_t getNumberOfMotorsRaw(int * num) override
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    { std::size_t ax; auto ret = getAxes(ax); *num = static_cast<int>(ax); return ret; }
#else
    { return getAxes(num); }
#endif
    return_t getTemperatureRaw(int m, double * val) override
    { *val = 0.0; return ret_ok; }
    return_t getTemperaturesRaw(double * vals) override
    { return getTemperatureRaw(0, &vals[0]); }
    return_t getTemperatureLimitRaw(int m, double * temp) override
    { *temp = 0.0; return ret_ok; }
    return_t setTemperatureLimitRaw(int m, double temp) override
    { return ret_ok; }
    return_t getGearboxRatioRaw(int m, double * val) override
    { *val = 0.0; return ret_ok; }
    return_t setGearboxRatioRaw(int m, double val) override
    { return ret_ok; }

    //  --------- IMotorEncodersRaw declarations ---------

    return_t getNumberOfMotorEncodersRaw(int * num) override
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    { std::size_t ax; auto ret = getAxes(ax); *num = static_cast<int>(ax); return ret; }
#else
    { return getAxes(num); }
#endif
    return_t resetMotorEncoderRaw(int m) override
    { return ret_ok; }
    return_t resetMotorEncodersRaw() override
    { return ret_ok; }
    return_t setMotorEncoderCountsPerRevolutionRaw(int m, double cpr) override
    { return ret_ok; }
    return_t getMotorEncoderCountsPerRevolutionRaw(int m, double * cpr) override
    { *cpr = 0.0; return ret_ok; }
    return_t setMotorEncoderRaw(int m, double val) override
    { return ret_ok; }
    return_t setMotorEncodersRaw(const double * vals) override
    { return ret_ok; }
    return_t getMotorEncoderRaw(int m, double * v) override
    { *v = 0.0; return ret_ok; }
    return_t getMotorEncodersRaw(double * encs) override
    { return getMotorEncoderRaw(0, &encs[0]); }
    return_t getMotorEncoderTimedRaw(int m, double * enc, double * stamp) override
    { *enc = 0.0; *stamp = yarp::os::SystemClock::nowSystem(); return ret_ok; }
    return_t getMotorEncodersTimedRaw(double * encs, double * stamps) override
    { return getMotorEncoderTimedRaw(0, &encs[0], &stamps[0]); }
    return_t getMotorEncoderSpeedRaw(int m, double * sp) override
    { *sp = 0.0; return ret_ok; }
    return_t getMotorEncoderSpeedsRaw(double *spds) override
    { return getMotorEncoderSpeedRaw(0, &spds[0]); }
    return_t getMotorEncoderAccelerationRaw(int m, double * acc) override
    { *acc = 0.0; return ret_ok; }
    return_t getMotorEncoderAccelerationsRaw(double * accs) override
    { return getMotorEncoderAccelerationRaw(0, &accs[0]); }

    //  --------- IPidControlRaw declarations ---------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return_t getAvailablePidsRaw(int j, std::vector<yarp::dev::PidControlTypeEnum> & avail) override
    { /* too many to list them all here */ return ret_ok; }
#endif
    return_t setPidRaw(const yarp::dev::PidControlTypeEnum & pidtype, int j, const yarp::dev::Pid & pid) override
    { return ret_ok; }
    return_t setPidsRaw(const yarp::dev::PidControlTypeEnum & pidtype, const yarp::dev::Pid * pids) override
    { return ret_ok; }
    return_t setPidReferenceRaw(const yarp::dev::PidControlTypeEnum & pidtype, int j, double ref) override
    { return ret_ok; }
    return_t setPidReferencesRaw(const yarp::dev::PidControlTypeEnum & pidtype, const double * refs) override
    { return ret_ok; }
    return_t setPidErrorLimitRaw(const yarp::dev::PidControlTypeEnum & pidtype, int j, double limit) override
    { return ret_ok; }
    return_t setPidErrorLimitsRaw(const yarp::dev::PidControlTypeEnum & pidtype, const double * limits) override
    { return ret_ok; }
    return_t setPidOffsetRaw(const yarp::dev::PidControlTypeEnum & pidtype, int j, double v) override
    { return ret_ok; }
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return_t setPidFeedforwardRaw(const yarp::dev::PidControlTypeEnum & pidtype, int j, double v) override
    { return ret_ok; }
#endif
    return_t getPidErrorRaw(const yarp::dev::PidControlTypeEnum & pidtype, int j, double * err) override
    { *err = 0.0; return ret_ok; }
    return_t getPidErrorsRaw(const yarp::dev::PidControlTypeEnum & pidtype, double * errs) override
    { return getPidErrorRaw(pidtype, 0, &errs[0]); }
    return_t getPidOutputRaw(const yarp::dev::PidControlTypeEnum & pidtype, int j, double * out) override
    { *out = 0.0; return ret_ok; }
    return_t getPidOutputsRaw(const yarp::dev::PidControlTypeEnum & pidtype, double * outs) override
    { return getPidOutputRaw(pidtype, 0, &outs[0]); }
    return_t getPidRaw(const yarp::dev::PidControlTypeEnum & pidtype, int j, yarp::dev::Pid * pid) override
    { return ret_ok; }
    return_t getPidsRaw(const yarp::dev::PidControlTypeEnum & pidtype, yarp::dev::Pid * pids) override
    { return ret_ok; }
    return_t getPidReferenceRaw(const yarp::dev::PidControlTypeEnum & pidtype, int j, double * ref) override
    { *ref = 0.0; return ret_ok; }
    return_t getPidReferencesRaw(const yarp::dev::PidControlTypeEnum & pidtype, double * refs) override
    { return getPidReferenceRaw(pidtype, 0, &refs[0]); }
    return_t getPidErrorLimitRaw(const yarp::dev::PidControlTypeEnum & pidtype, int j, double * limit) override
    { *limit = 0.0; return ret_ok; }
    return_t getPidErrorLimitsRaw(const yarp::dev::PidControlTypeEnum & pidtype, double * limits) override
    { return getPidErrorLimitRaw(pidtype, 0, &limits[0]); }
    return_t resetPidRaw(const yarp::dev::PidControlTypeEnum & pidtype, int j) override
    { return ret_ok; }
    return_t disablePidRaw(const yarp::dev::PidControlTypeEnum & pidtype, int j) override
    { return ret_ok; }
    return_t enablePidRaw(const yarp::dev::PidControlTypeEnum & pidtype, int j) override
    { return ret_ok; }
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return_t getPidOffsetRaw(const yarp::dev::PidControlTypeEnum & pidtype, int j, double & v) override
    { v = 0.0; return ret_ok; }
    return_t getPidFeedforwardRaw(const yarp::dev::PidControlTypeEnum & pidtype, int j, double & v) override
    { return ret_ok; }
    return_t isPidEnabledRaw(const yarp::dev::PidControlTypeEnum & pidtype, int j, bool & enabled) override
    { enabled = true; return ret_ok; }
    return_t getPidExtraInfoRaw(const yarp::dev::PidControlTypeEnum & pidtype, int j, yarp::dev::PidExtraInfo & info) override
    { return ret_ok; }
    return_t getPidExtraInfosRaw(const yarp::dev::PidControlTypeEnum & pidtype, std::vector<yarp::dev::PidExtraInfo> & info) override
    { return ret_ok; }
#else
    return_t isPidEnabledRaw(const yarp::dev::PidControlTypeEnum & pidtype, int j, bool * enabled) override
    { *enabled = true; return ret_ok; }
#endif

    // ------- IPositionControlRaw declarations -------

    //return_t getAxesRaw(int * ax) override;
    return_t positionMoveRaw(int j, double ref) override
    { return ret_ok; }
    return_t positionMoveRaw(const double * refs) override
    { return ret_ok; }
    return_t positionMoveRaw(int n_joint, const int * joints, const double * refs) override
    { return ret_ok; }
    return_t relativeMoveRaw(int j, double delta) override
    { return ret_ok; }
    return_t relativeMoveRaw(const double * deltas) override
    { return ret_ok; }
    return_t relativeMoveRaw(int n_joint, const int * joints, const double * deltas) override
    { return ret_ok; }
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return_t checkMotionDoneRaw(int j, bool & flag) override
    { flag = true; return ret_ok; }
    return_t checkMotionDoneRaw(bool & flag) override
    { return checkMotionDoneRaw(0, flag); }
    return_t checkMotionDoneRaw(const std::vector<int> & joints, bool & flag) override
    { return checkMotionDoneRaw(0, flag); }
    return_t setTrajSpeedRaw(int j, double sp) override
    { return ret_ok; }
    return_t setTrajSpeedsRaw(const double * spds) override
    { return ret_ok; }
    return_t setTrajSpeedsRaw(int n_joint, const int * joints, const double * spds) override
    { return ret_ok; }
    return_t setTrajAccelerationRaw(int j, double acc) override
    { return ret_ok; }
    return_t setTrajAccelerationsRaw(const double * accs) override
    { return ret_ok; }
    return_t setTrajAccelerationsRaw(int n_joint, const int * joints, const double * accs) override
    { return ret_ok; }
    return_t getTrajSpeedRaw(int j, double * spd) override
    { *spd = 0.0; return ret_ok; }
    return_t getTrajSpeedsRaw(double * spds) override
    { return getTrajSpeedRaw(0, &spds[0]); }
    return_t getTrajSpeedsRaw(int n_joint, const int * joints, double * spds) override
    { return getTrajSpeedRaw(0, &spds[0]); }
    return_t getTrajAccelerationRaw(int j, double * acc) override
    { *acc = 0.0; return ret_ok; }
    return_t getTrajAccelerationsRaw(double * accs) override
    { return getTrajAccelerationRaw(0, &accs[0]); }
    return_t getTrajAccelerationsRaw(int n_joint, const int * joints, double * accs) override
    { return getTrajAccelerationRaw(0, &accs[0]); }
#else
    return_t checkMotionDoneRaw(int j, bool * flag) override
    { *flag = true; return ret_ok; }
    return_t checkMotionDoneRaw(bool * flag) override
    { return checkMotionDoneRaw(0, flag); }
    return_t checkMotionDoneRaw(int n_joint, const int * joints, bool * flag) override
    { return checkMotionDoneRaw(0, flag); }
    return_t setRefSpeedRaw(int j, double sp) override
    { return ret_ok; }
    return_t setRefSpeedsRaw(const double * spds) override
    { return ret_ok; }
    return_t setRefSpeedsRaw(int n_joint, const int * joints, const double * spds) override
    { return ret_ok; }
    return_t setRefAccelerationRaw(int j, double acc) override
    { return ret_ok; }
    return_t setRefAccelerationsRaw(const double * accs) override
    { return ret_ok; }
    return_t setRefAccelerationsRaw(int n_joint, const int * joints, const double * accs) override
    { return ret_ok; }
    return_t getRefSpeedRaw(int j, double * spd) override
    { *spd = 0.0; return ret_ok; }
    return_t getRefSpeedsRaw(double * spds) override
    { return getRefSpeedRaw(0, &spds[0]); }
    return_t getRefSpeedsRaw(int n_joint, const int * joints, double * spds) override
    { return getRefSpeedRaw(0, &spds[0]); }
    return_t getRefAccelerationRaw(int j, double * acc) override
    { *acc = 0.0; return ret_ok; }
    return_t getRefAccelerationsRaw(double * accs) override
    { return getRefAccelerationRaw(0, &accs[0]); }
    return_t getRefAccelerationsRaw(int n_joint, const int * joints, double * accs) override
    { return getRefAccelerationRaw(0, &accs[0]); }
#endif
    return_t stopRaw(int j) override
    { return ret_ok; }
    return_t stopRaw() override
    { return ret_ok; }
    return_t stopRaw(int n_joint, const int *joints) override
    { return ret_ok; }
    return_t getTargetPositionRaw(int joint, double * ref) override
    { *ref = 0.0; return ret_ok; }
    return_t getTargetPositionsRaw(double * refs) override
    { return getTargetPositionRaw(0, &refs[0]); }
    return_t getTargetPositionsRaw(int n_joint, const int * joints, double * refs) override
    { return getTargetPositionRaw(0, &refs[0]); }

    // ------- IPositionDirectRaw declarations -------

    return_t setPositionRaw(int j, double ref) override
    { return ret_ok; }
    return_t setPositionsRaw(const double * refs) override
    { return ret_ok; }
    return_t setPositionsRaw(int n_joint, const int * joints, const double * refs) override
    { return ret_ok; }
    return_t getRefPositionRaw(int joint, double * ref) override
    { *ref = 0.0; return ret_ok; }
    return_t getRefPositionsRaw(double * refs) override
    { return getRefPositionRaw(0, &refs[0]); }
    return_t getRefPositionsRaw(int n_joint, const int * joints, double * refs) override
    { return getRefPositionRaw(0, &refs[0]); }

    //  --------- IPWMControl declarations ---------

    return_t setRefDutyCycleRaw(int m, double ref) override
    { return ret_ok; }
    return_t setRefDutyCyclesRaw(const double * refs) override
    { return ret_ok; }
    return_t getRefDutyCycleRaw(int m, double * ref) override
    { *ref = 0.0; return ret_ok; }
    return_t getRefDutyCyclesRaw(double * refs) override
    { return getRefDutyCycleRaw(0, &refs[0]); }
    return_t getDutyCycleRaw(int m, double * val) override
    { *val = 0.0; return ret_ok; }
    return_t getDutyCyclesRaw(double * vals) override
    { return getDutyCycleRaw(0, &vals[0]); }

    // ------- IRemoteVariablesRaw declarations -------

    return_t getRemoteVariableRaw(std::string key, yarp::os::Bottle & val) override
    { return ret_ok; }
    return_t setRemoteVariableRaw(std::string key, const yarp::os::Bottle & val) override
    { return ret_ok; }
    return_t getRemoteVariablesListRaw(yarp::os::Bottle * listOfKeys) override
    { return ret_ok; }

    // -------- ITorqueControlRaw declarations --------

    return_t getRefTorqueRaw(int j, double * t) override
    { *t = 0.0; return ret_ok; }
    return_t getRefTorquesRaw(double * t) override
    { return getRefTorqueRaw(0, &t[0]); }
    return_t setRefTorqueRaw(int j, double t) override
    { return ret_ok; }
    return_t setRefTorquesRaw(const double * t) override
    { return ret_ok; }
    return_t setRefTorquesRaw(int n_joint, const int * joints, const double * t) override
    { return ret_ok; }
    return_t getMotorTorqueParamsRaw(int j, yarp::dev::MotorTorqueParameters * params) override
    { return ret_ok; }
    return_t setMotorTorqueParamsRaw(int j, const yarp::dev::MotorTorqueParameters params) override
    { return ret_ok; }
    return_t getTorqueRaw(int j, double * t) override
    { *t = 0.0; return ret_ok; }
    return_t getTorquesRaw(double * t) override
    { return getTorqueRaw(0, &t[0]); }
    return_t getTorqueRangeRaw(int j, double * min, double * max) override
    { *min = *max = 0.0; return ret_ok; }
    return_t getTorqueRangesRaw(double * mins, double * maxs) override
    { return getTorqueRangeRaw(0, &mins[0], &maxs[0]); }

    //  --------- IVelocityControl declarations ---------

    return_t velocityMoveRaw(int j, double spd) override
    { return ret_ok; }
    return_t velocityMoveRaw(const double * spds) override
    { return ret_ok; }
    return_t velocityMoveRaw(int n_joint, const int * joints, const double * spds) override
    { return ret_ok; }
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return_t getTargetVelocityRaw(int joint, double * vel) override
    { *vel = 0.0; return ret_ok; }
    return_t getTargetVelocitiesRaw(double * vels) override
    { return getTargetVelocityRaw(0, &vels[0]); }
    return_t getTargetVelocitiesRaw(int n_joint, const int * joints, double * vels) override
    { return getTargetVelocityRaw(0, &vels[0]); }
#else
    return_t getRefVelocityRaw(int joint, double * vel) override
    { *vel = 0.0; return ret_ok; }
    return_t getRefVelocitiesRaw(double * vels) override
    { return getRefVelocityRaw(0, &vels[0]); }
    return_t getRefVelocitiesRaw(int n_joint, const int * joints, double * vels) override
    { return getRefVelocityRaw(0, &vels[0]); }
#endif

private:
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    yarp::dev::ControlModeEnum controlMode {VOCAB_CM_CONFIGURED};
#else
    int controlMode {VOCAB_CM_CONFIGURED};
#endif
    yarp::dev::InteractionModeEnum interactionMode {yarp::dev::VOCAB_IM_UNKNOWN};
};

#endif // __FAKE_JOINT_HPP__
