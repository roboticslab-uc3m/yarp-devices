// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __TECHNOSOFT_IPOS_BASE_HPP__
#define __TECHNOSOFT_IPOS_BASE_HPP__

#include <cstdint>

#include <atomic>
#include <bitset>
#include <memory>
#include <string>

#include <yarp/conf/numeric.h>
#include <yarp/conf/version.h>

#include <yarp/os/Timer.h>

#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/IAxisInfo.h>
#include <yarp/dev/IControlLimits.h>
#include <yarp/dev/IControlMode.h>
#include <yarp/dev/ICurrentControl.h>
#include <yarp/dev/IEncodersTimed.h>
#include <yarp/dev/IImpedanceControl.h>
#include <yarp/dev/IInteractionMode.h>
#include <yarp/dev/IJointFault.h>
#include <yarp/dev/IMotor.h>
#include <yarp/dev/IMotorEncoders.h>
#include <yarp/dev/IPidControl.h>
#include <yarp/dev/IPositionControl.h>
#include <yarp/dev/IPositionDirect.h>
#include <yarp/dev/IRemoteVariables.h>
#include <yarp/dev/ITorqueControl.h>
#include <yarp/dev/IVelocityControl.h>
#include <yarp/dev/PolyDriver.h>

#include "CanOpenNode.hpp"
#include "CommandBuffer.hpp"
#include "EncoderRead.hpp"
#include "ICanBusSharer.hpp"
#include "PdoProtocol.hpp"
#include "StateObserver.hpp"

#include "TechnosoftIpos_ParamsParser.h"

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
#define CHECK_JOINT(j) do { if (std::size_t ax; getAxes(ax), (j) < 0 || (j) >= ax) return yarp::dev::ReturnValue::return_code::return_value_error_input_out_of_bounds; } while (0)
#define CHECK_MODE(mode) do { if ((mode) != actualControlMode) return yarp::dev::ReturnValue::return_code::return_value_error_not_ready; } while (0)
#else
#define CHECK_JOINT(j) do { if (int ax; getAxes(&ax), (j) < 0 || (j) >= ax) return false; } while (0)
#define CHECK_MODE(mode) do { if ((mode) != actualControlMode) return false; } while (0)
#endif

namespace roboticslab
{

/**
 * @ingroup TechnosoftIpos
 * @brief Base class for all proxied TechnosoftIpos implementations.
 */
class TechnosoftIposBase : public yarp::dev::DeviceDriver,
                           public yarp::dev::IAxisInfoRaw,
                           public yarp::dev::IControlLimitsRaw,
                           public yarp::dev::IControlModeRaw,
                           public yarp::dev::ICurrentControlRaw,
                           public yarp::dev::IEncodersTimedRaw,
                           public yarp::dev::IImpedanceControlRaw,
                           public yarp::dev::IInteractionModeRaw,
                           public yarp::dev::IJointFaultRaw,
                           public yarp::dev::IMotorRaw,
                           public yarp::dev::IMotorEncodersRaw,
                           public yarp::dev::IPidControlRaw,
                           public yarp::dev::IPositionControlRaw,
                           public yarp::dev::IPositionDirectRaw,
                           public yarp::dev::IRemoteVariablesRaw,
                           public yarp::dev::ITorqueControlRaw,
                           public yarp::dev::IVelocityControlRaw,
                           public ICanBusSharer
{
public:

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    using return_t = yarp::dev::ReturnValue;

    constexpr static auto ret_not_ok = return_t::return_code::return_value_error_not_implemented_by_device;
#else
    using return_t = bool;

    constexpr static auto ret_not_ok = false;
#endif

    TechnosoftIposBase(const TechnosoftIpos_ParamsParser & _params)
        : params(_params)
    {}

    //  --------- DeviceDriver declarations. Implementation in DeviceDriverImpl.cpp ---------

    bool open(yarp::os::Searchable & config) override;
    bool close() override;

    //  --------- ICanBusSharer declarations. Implementation in ICanBusSharerImpl.cpp ---------

    unsigned int getId() override;
    std::vector<unsigned int> getAdditionalIds() override;
    bool notifyMessage(const can_message & message) override;
    bool initialize() override;
    bool finalize() override;
    bool registerSender(ICanSenderDelegate * sender) override;

    //  --------- IAxisInfoRaw declarations. Implementation in IAxisInfoRawImpl.cpp ---------

    return_t getAxisNameRaw(int axis, std::string & name) override;
    return_t getJointTypeRaw(int axis, yarp::dev::JointTypeEnum & type) override;

    //  --------- IControlLimitsRaw declarations. Implementation in IControlLimitsRawImpl.cpp ---------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return_t setPosLimitsRaw(int axis, double min, double max) override;
    return_t getPosLimitsRaw(int axis, double * min, double * max) override;
#else
    return_t setLimitsRaw(int axis, double min, double max) override;
    return_t getLimitsRaw(int axis, double * min, double * max) override;
#endif
    return_t setVelLimitsRaw(int axis, double min, double max) override;
    return_t getVelLimitsRaw(int axis, double * min, double * max) override;

    //  --------- IControlModeRaw declarations. Implementation in IControlModeRawImpl.cpp ---------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return_t getControlModesRaw(std::vector<yarp::dev::ControlModeEnum> & modes) override
    { return getControlModeRaw(0, modes[0]); }

    return_t getControlModesRaw(const std::vector<int> & joints, std::vector<yarp::dev::ControlModeEnum> & modes) override
    { return getControlModeRaw(joints[0], modes[0]); }

    return_t setControlModesRaw(const std::vector<yarp::dev::SelectableControlModeEnum> & modes) override
    { return setControlModeRaw(0, modes[0]); }

    return_t setControlModesRaw(const std::vector<int> & joints, const std::vector<yarp::dev::SelectableControlModeEnum> & modes) override
    { return setControlModeRaw(joints[0], modes[0]); }
#else
    return_t getControlModesRaw(int * modes) override
    { return getControlModeRaw(0, &modes[0]); }

    return_t getControlModesRaw(int n_joint, const int * joints, int * modes) override
    { return getControlModeRaw(joints[0], &modes[0]); }

    return_t setControlModesRaw(int * modes) override
    { return setControlModeRaw(0, modes[0]); }

    return_t setControlModesRaw(int n_joint, const int * joints, int * modes) override
    { return setControlModeRaw(joints[0], modes[0]); }
#endif

    //  --------- ICurrentControlRaw declarations. Implementation in ICurrentControlRawImpl.cpp ---------

    return_t getCurrentRaw(int m, double * curr) override;

    return_t getCurrentsRaw(double * currs) override
    { return getCurrentRaw(0, &currs[0]); }

    return_t getCurrentRangeRaw(int m, double * min, double * max) override;

    return_t getCurrentRangesRaw(double * min, double * max) override
    { return getCurrentRangeRaw(0, min, max); }

    return_t setRefCurrentRaw(int m, double curr) override;

    return_t setRefCurrentsRaw(const double * currs) override
    { return setRefCurrentRaw(0, currs[0]); }

    return_t setRefCurrentsRaw(int n_motor, const int * motors, const double * currs) override
    { return setRefCurrentRaw(motors[0], currs[0]); }

    return_t getRefCurrentRaw(int m, double * curr) override;

    return_t getRefCurrentsRaw(double * currs) override
    { return getRefCurrentRaw(0, &currs[0]); }

    //  ---------- IEncodersRaw declarations. Implementation in IEncodersRawImpl.cpp ----------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return_t getAxes(std::size_t & ax) override;
#else
    return_t getAxes(int * ax) override;
#endif

    return_t resetEncoderRaw(int j) override;

    return_t resetEncodersRaw() override
    { return resetEncoderRaw(0); }

    return_t setEncoderRaw(int j, double val) override;

    return_t setEncodersRaw(const double * vals) override
    { return setEncoderRaw(0, vals[0]); }

    return_t getEncoderRaw(int j, double * v) override;

    return_t getEncodersRaw(double * encs) override
    { return getEncoderRaw(0, &encs[0]); }

    return_t getEncoderSpeedRaw(int j, double * sp) override;

    return_t getEncoderSpeedsRaw(double * spds) override
    { return getEncoderSpeedRaw(0, &spds[0]); }

    return_t getEncoderAccelerationRaw(int j, double * spds) override;

    return_t getEncoderAccelerationsRaw(double * accs) override
    { return getEncoderAccelerationRaw(0, &accs[0]); }

    return_t getEncoderTimedRaw(int j, double * encs, double * time) override;

    return_t getEncodersTimedRaw(double * encs, double * times) override
    { return getEncoderTimedRaw(0, &encs[0], &times[0]); }

    //  ---------- IImpedanceControlRaw declarations. Implementation in IImpedanceControlRawImpl.cpp ----------

    return_t getImpedanceRaw(int j, double * stiffness, double * damping) override
    { return ret_not_ok; }

    return_t setImpedanceRaw(int j, double stiffness, double damping) override
    { return ret_not_ok; }

    return_t setImpedanceOffsetRaw(int j, double offset) override
    { return ret_not_ok; }

    return_t getImpedanceOffsetRaw(int j, double * offset) override
    { return ret_not_ok; }

    return_t getCurrentImpedanceLimitRaw(int j, double * min_stiff, double * max_stiff, double * min_damp, double * max_damp) override
    { return ret_not_ok; }

    //  ---------- IInteractionModeRaw declarations. Implementation in IInteractionModeRawImpl.cpp ----------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return_t getInteractionModeRaw(int axis, yarp::dev::InteractionModeEnum & mode) override
    { return ret_not_ok; }

    return_t getInteractionModesRaw(std::vector<yarp::dev::InteractionModeEnum> & modes) override
    { return getInteractionModeRaw(0, modes[0]); }

    return_t getInteractionModesRaw(const std::vector<int> & joints, std::vector<yarp::dev::InteractionModeEnum> & modes) override
    { return getInteractionModeRaw(joints[0], modes[0]); }

    return_t setInteractionModeRaw(int axis, yarp::dev::InteractionModeEnum mode) override
    { return ret_not_ok; }

    return_t setInteractionModesRaw(const std::vector<yarp::dev::InteractionModeEnum> & modes) override
    { return setInteractionModeRaw(0, modes[0]); }

    return_t setInteractionModesRaw(const std::vector<int> & joints, const std::vector<yarp::dev::InteractionModeEnum> & modes) override
    { return setInteractionModeRaw(joints[0], modes[0]); }
#else
    return_t getInteractionModeRaw(int axis, yarp::dev::InteractionModeEnum * mode) override
    { return ret_not_ok; }

    return_t getInteractionModesRaw(int n_joints, int * joints, yarp::dev::InteractionModeEnum * modes) override
    { return getInteractionModeRaw(joints[0], &modes[0]); }

    return_t getInteractionModesRaw(yarp::dev::InteractionModeEnum * modes) override
    { return getInteractionModeRaw(0, &modes[0]); }

    return_t setInteractionModeRaw(int axis, yarp::dev::InteractionModeEnum mode) override
    { return ret_not_ok; }

    return_t setInteractionModesRaw(int n_joints, int * joints, yarp::dev::InteractionModeEnum * modes) override
    { return setInteractionModeRaw(joints[0], modes[0]); }

    return_t setInteractionModesRaw(yarp::dev::InteractionModeEnum * modes) override
    { return setInteractionModeRaw(0, modes[0]); }
#endif

    //  ---------- IJointFaultRaw declarations. Implementation in IJointFaultRawImpl.cpp ----------

    return_t getLastJointFaultRaw(int j, int & fault, std::string & message) override;

    //  --------- IMotorRaw declarations. Implementation in IMotorRawImpl.cpp ---------

    return_t getNumberOfMotorsRaw(int * num) override;
    return_t getTemperatureRaw(int m, double * val) override;
    return_t getTemperaturesRaw(double * vals) override;
    return_t getTemperatureLimitRaw(int m, double * temp) override;
    return_t setTemperatureLimitRaw(int m, double temp) override;
    return_t getGearboxRatioRaw(int m, double * val) override;
    return_t setGearboxRatioRaw(int m, double val) override;

    //  --------- IMotorEncodersRaw declarations. Implementation in IMotorEncodersRawImpl.cpp ---------

    return_t getNumberOfMotorEncodersRaw(int * num) override;
    return_t resetMotorEncoderRaw(int m) override;

    return_t resetMotorEncodersRaw() override
    { return resetMotorEncoderRaw(0); }

    return_t setMotorEncoderCountsPerRevolutionRaw(int m, double cpr) override;
    return_t getMotorEncoderCountsPerRevolutionRaw(int m, double * cpr) override;
    return_t setMotorEncoderRaw(int m, double val) override;

    return_t setMotorEncodersRaw(const double * vals) override
    { return setMotorEncoderRaw(0, vals[0]); }

    return_t getMotorEncoderRaw(int m, double * v) override;

    return_t getMotorEncodersRaw(double * encs) override
    { return getMotorEncoderSpeedRaw(0, &encs[0]); }

    return_t getMotorEncoderTimedRaw(int m, double * encs, double * stamp) override;

    return_t getMotorEncodersTimedRaw(double * encs, double * stamps) override
    { return getMotorEncoderTimedRaw(0, &encs[0], &stamps[0]); }

    return_t getMotorEncoderSpeedRaw(int m, double * sp) override;

    return_t getMotorEncoderSpeedsRaw(double * spds) override
    { return getMotorEncoderSpeedRaw(0, &spds[0]); }

    return_t getMotorEncoderAccelerationRaw(int m, double * spds) override;

    return_t getMotorEncoderAccelerationsRaw(double * accs) override
    { return getMotorEncoderAccelerationRaw(0, &accs[0]); }

    //  --------- IPidControlRaw declarations. Implementation in IPidControlRawImpl.cpp ---------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return_t getAvailablePidsRaw(int j, std::vector<yarp::dev::PidControlTypeEnum> & avail) override
    { return ret_not_ok; }
#endif

    return_t setPidRaw(const yarp::dev::PidControlTypeEnum & pidtype, int j, const yarp::dev::Pid & pid) override
    { return ret_not_ok; }

    return_t setPidsRaw(const yarp::dev::PidControlTypeEnum & pidtype, const yarp::dev::Pid * pids) override
    { return setPidRaw(pidtype, 0, pids[0]); }

    return_t setPidReferenceRaw(const yarp::dev::PidControlTypeEnum & pidtype, int j, double ref) override
    { return ret_not_ok; }

    return_t setPidReferencesRaw(const yarp::dev::PidControlTypeEnum & pidtype, const double * refs) override
    { return setPidReferenceRaw(pidtype, 0, refs[0]); }

    return_t setPidErrorLimitRaw(const yarp::dev::PidControlTypeEnum & pidtype, int j, double limit) override
    { return ret_not_ok; }

    return_t setPidErrorLimitsRaw(const yarp::dev::PidControlTypeEnum & pidtype, const double * limits) override
    { return setPidErrorLimitRaw(pidtype, 0, limits[0]); }

    return_t getPidErrorRaw(const yarp::dev::PidControlTypeEnum & pidtype, int j, double * err) override
    { return ret_not_ok; }

    return_t getPidErrorsRaw(const yarp::dev::PidControlTypeEnum & pidtype, double * errs) override
    { return getPidErrorRaw(pidtype, 0, &errs[0]); }

    return_t getPidOutputRaw(const yarp::dev::PidControlTypeEnum & pidtype, int j, double * out) override
    { return ret_not_ok; }

    return_t getPidOutputsRaw(const yarp::dev::PidControlTypeEnum & pidtype, double * outs) override
    { return getPidOutputRaw(pidtype, 0, &outs[0]); }

    return_t getPidRaw(const yarp::dev::PidControlTypeEnum & pidtype, int j, yarp::dev::Pid * pid) override
    { return ret_not_ok; }

    return_t getPidsRaw(const yarp::dev::PidControlTypeEnum & pidtype, yarp::dev::Pid * pids) override
    { return getPidRaw(pidtype, 0, &pids[0]); }

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return_t getPidOffsetRaw(const yarp::dev::PidControlTypeEnum & pidtype, int j, double & v) override
    { return ret_not_ok; }

    return_t getPidFeedforwardRaw(const yarp::dev::PidControlTypeEnum & pidtype, int j, double & v) override
    { return ret_not_ok; }

    return_t getPidExtraInfoRaw(const yarp::dev::PidControlTypeEnum & pidtype, int j, yarp::dev::PidExtraInfo & info) override
    { return ret_not_ok; }

    return_t getPidExtraInfosRaw(const yarp::dev::PidControlTypeEnum & pidtype, std::vector<yarp::dev::PidExtraInfo> & info) override
    { return ret_not_ok; }
#endif

    return_t getPidReferenceRaw(const yarp::dev::PidControlTypeEnum & pidtype, int j, double * ref) override
    { return ret_not_ok; }

    return_t getPidReferencesRaw(const yarp::dev::PidControlTypeEnum & pidtype, double * refs) override
    { return getPidReferenceRaw(pidtype, 0, &refs[0]); }

    return_t getPidErrorLimitRaw(const yarp::dev::PidControlTypeEnum & pidtype, int j, double * limit) override
    { return ret_not_ok; }

    return_t getPidErrorLimitsRaw(const yarp::dev::PidControlTypeEnum & pidtype, double * limits) override
    { return getPidErrorLimitRaw(pidtype, 0, &limits[0]); }

    return_t resetPidRaw(const yarp::dev::PidControlTypeEnum & pidtype, int j) override
    { return ret_not_ok; }

    return_t disablePidRaw(const yarp::dev::PidControlTypeEnum & pidtype, int j) override
    { return ret_not_ok; }

    return_t enablePidRaw(const yarp::dev::PidControlTypeEnum & pidtype, int j) override
    { return ret_not_ok; }

    return_t setPidOffsetRaw(const yarp::dev::PidControlTypeEnum & pidtype, int j, double v) override
    { return ret_not_ok; }

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return_t setPidFeedforwardRaw(const yarp::dev::PidControlTypeEnum & pidtype, int j, double v) override
    { return ret_not_ok; }

    return_t isPidEnabledRaw(const yarp::dev::PidControlTypeEnum & pidtype, int j, bool & enabled) override
    { return ret_not_ok; }
#else
    return_t isPidEnabledRaw(const yarp::dev::PidControlTypeEnum & pidtype, int j, bool * enabled) override
    { return ret_not_ok; }
#endif

    // ------- IPositionControlRaw declarations. Implementation in IPositionControlRawImpl.cpp -------

    using yarp::dev::IPositionControlRaw::positionMoveRaw;

    return_t positionMoveRaw(const double * refs) override
    { return positionMoveRaw(0, refs[0]); }

    return_t positionMoveRaw(int n_joint, const int * joints, const double * refs) override
    { return positionMoveRaw(joints[0], refs[0]); }

    using yarp::dev::IPositionControlRaw::relativeMoveRaw;

    return_t relativeMoveRaw(const double * deltas) override
    { return relativeMoveRaw(0, deltas[0]); }

    return_t relativeMoveRaw(int n_joint, const int * joints, const double * deltas) override
    { return relativeMoveRaw(joints[0], deltas[0]); }

    using yarp::dev::IPositionControlRaw::checkMotionDoneRaw;

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return_t checkMotionDoneRaw(bool & flag) override
    { return checkMotionDoneRaw(0, flag); }

    return_t checkMotionDoneRaw(const std::vector<int> & joints, bool & flag) override
    { return checkMotionDoneRaw(joints[0], flag); }

    return_t setTrajSpeedsRaw(const double * spds) override
    { return setTrajSpeedRaw(0, spds[0]); }

    return_t setTrajSpeedsRaw(int n_joint, const int * joints, const double * spds) override
    { return setTrajSpeedRaw(joints[0], spds[0]); }

    using yarp::dev::IPositionControlRaw::setTrajAccelerationRaw;

    return_t setTrajAccelerationsRaw(const double * accs) override
    { return setTrajAccelerationRaw(0, accs[0]); }

    return_t setTrajAccelerationsRaw(int n_joint, const int * joints, const double * accs) override
    { return setTrajAccelerationRaw(joints[0], accs[0]); }

    return_t getTrajSpeedsRaw(double * spds) override
    { return getTrajSpeedRaw(0, &spds[0]); }

    return_t getTrajSpeedsRaw(int n_joint, const int * joints, double * spds) override
    { return getTrajSpeedRaw(joints[0], &spds[0]); }

    using yarp::dev::IPositionControlRaw::getTrajAccelerationRaw;

    return_t getTrajAccelerationsRaw(double * accs) override
    { return getTrajAccelerationRaw(0, &accs[0]); }

    return_t getTrajAccelerationsRaw(int n_joint, const int * joints, double * accs) override
    { return getTrajAccelerationRaw(joints[0], &accs[0]); }
#else
    return_t checkMotionDoneRaw(bool * flag) override
    { return checkMotionDoneRaw(0, flag); }

    return_t checkMotionDoneRaw(int n_joint, const int * joints, bool * flag) override
    { return checkMotionDoneRaw(joints[0], flag); }

    return_t setRefSpeedsRaw(const double * spds) override
    { return setRefSpeedRaw(0, spds[0]); }

    return_t setRefSpeedsRaw(int n_joint, const int * joints, const double * spds) override
    { return setRefSpeedRaw(joints[0], spds[0]); }

    using yarp::dev::IPositionControlRaw::setRefAccelerationRaw;

    return_t setRefAccelerationsRaw(const double * accs) override
    { return setRefAccelerationRaw(0, accs[0]); }

    return_t setRefAccelerationsRaw(int n_joint, const int * joints, const double * accs) override
    { return setRefAccelerationRaw(joints[0], accs[0]); }

    return_t getRefSpeedsRaw(double * spds) override
    { return getRefSpeedRaw(0, &spds[0]); }

    return_t getRefSpeedsRaw(int n_joint, const int * joints, double * spds) override
    { return getRefSpeedRaw(joints[0], &spds[0]); }

    using yarp::dev::IPositionControlRaw::getRefAccelerationRaw;

    return_t getRefAccelerationsRaw(double * accs) override
    { return getRefAccelerationRaw(0, &accs[0]); }

    return_t getRefAccelerationsRaw(int n_joint, const int * joints, double * accs) override
    { return getRefAccelerationRaw(joints[0], &accs[0]); }
#endif

    using yarp::dev::IPositionControlRaw::stopRaw;

    return_t stopRaw() override
    { return stopRaw(0); }

    return_t stopRaw(int n_joint, const int * joints) override
    { return stopRaw(joints[0]); }

    return_t getTargetPositionsRaw(double * refs) override
    { return getTargetPositionRaw(0, &refs[0]); }

    return_t getTargetPositionsRaw(int n_joint, const int * joints, double * refs) override
    { return getTargetPositionRaw(joints[0], &refs[0]); }

    // ------- IPositionDirectRaw declarations. Implementation in IPositionDirectRawImpl.cpp -------

    return_t setPositionsRaw(const double * refs) override
    { return setPositionRaw(0, refs[0]); }

    return_t setPositionsRaw(int n_joint, const int * joints, const double * refs) override
    { return setPositionRaw(joints[0], refs[0]); }

    return_t getRefPositionsRaw(double * refs) override
    { return getRefPositionRaw(0, &refs[0]); }

    return_t getRefPositionsRaw(int n_joint, const int * joints, double * refs) override
    { return getRefPositionRaw(joints[0], &refs[0]); }

    // -------- ITorqueControlRaw declarations. Implementation in ITorqueControlRawImpl.cpp --------

    return_t getRefTorqueRaw(int j, double * t) override;

    return_t getRefTorquesRaw(double * t) override
    { return getRefTorqueRaw(0, &t[0]); }

    return_t setRefTorqueRaw(int j, double t) override;

    return_t setRefTorquesRaw(int n_joint, const int * joints, const double * t) override
    { return setRefTorqueRaw(joints[0], t[0]); }

    return_t setRefTorquesRaw(const double * t) override
    { return setRefTorqueRaw(0, t[0]); }

    return_t getTorqueRaw(int j, double * t) override;

    return_t getTorquesRaw(double * t) override
    { return getTorqueRaw(0, &t[0]); }

    return_t getTorqueRangeRaw(int j, double * min, double * max) override;

    return_t getTorqueRangesRaw(double * min, double * max) override
    { return getTorqueRangeRaw(0, &min[0], &max[0]); }

    return_t getMotorTorqueParamsRaw(int j, yarp::dev::MotorTorqueParameters * params) override;
    return_t setMotorTorqueParamsRaw(int j, const yarp::dev::MotorTorqueParameters params) override;

    //  --------- IVelocityControlRaw declarations. Implementation in IVelocityControlRawImpl.cpp ---------

    using yarp::dev::IVelocityControlRaw::velocityMoveRaw;

    return_t velocityMoveRaw(const double * sp) override
    { return velocityMoveRaw(0, sp[0]); }

    return_t velocityMoveRaw(int n_joint, const int * joints, const double * spds) override
    { return velocityMoveRaw(joints[0], spds[0]); }

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return_t getTargetVelocitiesRaw(double * vels) override
    { return getTargetVelocityRaw(0, &vels[0]); }

    return_t getTargetVelocitiesRaw(int n_joint, const int * joints, double * vels) override
    { return getTargetVelocityRaw(joints[0], &vels[0]); }
#else
    return_t getRefVelocitiesRaw(double * vels) override
    { return getRefVelocityRaw(0, &vels[0]); }

    return_t getRefVelocitiesRaw(int n_joint, const int * joints, double * vels) override
    { return getRefVelocityRaw(joints[0], &vels[0]); }
#endif

protected:

    enum report_level { NONE, INFO, WARN, FAULT };
    enum limit_switch { POSITIVE, NEGATIVE, INACTIVE };

    struct report_storage
    {
        const char * reg;
        const std::bitset<16> & actual;
        const std::bitset<16> & stored;
    };

    bool reportBitToggle(report_storage report, int level, std::size_t pos, const char * msgSet, const char * msgReset = nullptr);

    virtual void interpretModesOfOperation(std::int8_t modesOfOperation) = 0;
    virtual void interpretIpStatus(std::uint16_t ipStatus) {}
    virtual void onPositionLimitTriggered() = 0;
    virtual void reset() = 0;

    //! Wait with timeout for requested control mode change.
    bool awaitControlMode(yarp::conf::vocab32_t mode);

    //! Convert position, speed or acceleration to internal units.
    double degreesToInternalUnits(double value, int derivativeOrder = 0) const;

    //! Convert position, speed or acceleration to degrees.
    double internalUnitsToDegrees(double value, int derivativeOrder = 0) const;

    //! Convert current to internal units.
    std::int16_t currentToInternalUnits(double value) const;

    //! Convert current to amperes.
    double internalUnitsToCurrent(std::int16_t value) const;

    //! Apply internal iPOS conversion to express drive peak current in amperes.
    double internalUnitsToPeakCurrent(std::int16_t value) const;

    //! Convert current (amperes) to torque (Nm).
    double currentToTorque(double current) const;

    //! Convert torque (Nm) to current (amperes).
    double torqueToCurrent(double torque) const;

    CanOpenNode * can {nullptr};
    CommandBuffer commandBuffer;

    std::unique_ptr<StateObserver> controlModeObserverPtr {new StateObserver(1.0)}; // arbitrary 1 second wait
    std::unique_ptr<EncoderRead> lastEncoderRead {nullptr};

    // read/write, no concurrent access

    std::bitset<16> msr;
    std::bitset<16> mer;
    std::bitset<16> der;
    std::bitset<16> der2;
    std::bitset<16> cer;

    std::int8_t modesOfOperation {0};

    DriveState driveState {DriveState::NOT_READY_TO_SWITCH_ON};

    bool configuredOnce {false};

    // read/write with atomic access

    std::atomic<std::int16_t> lastCurrentRead {0};

    std::atomic<yarp::conf::vocab32_t> actualControlMode {0};
    std::atomic<yarp::conf::vocab32_t> requestedcontrolMode {0};

    std::atomic<double> tr {0.0};
    std::atomic<double> k {0.0};
    std::atomic<int> encoderPulses {0};

    std::atomic<double> maxVel {0.0};
    std::atomic<double> min {0.0};
    std::atomic<double> max {0.0};
    std::atomic<double> refSpeed {0.0};
    std::atomic<double> refAcceleration {0.0};

    std::atomic<double> lastHeartbeat {0.0};
    std::atomic<std::uint8_t> lastNmtState {0};
    std::atomic<std::uint16_t> lastFaultCode {0};
    std::atomic<const char *> lastFaultMessage;

    std::atomic<limit_switch> limitSwitchState {INACTIVE};

    // read only after initial configuration, conceptually immutable

    yarp::conf::vocab32_t initialControlMode {0};
    yarp::conf::vocab32_t jointType {0};

    PdoConfiguration tpdo1Conf;
    PdoConfiguration tpdo2Conf;
    PdoConfiguration tpdo3Conf;

    const TechnosoftIpos_ParamsParser & params;

private:

    //! Make sure stored variables actually make sense.
    static bool validateInitialState(const TechnosoftIpos_ParamsParser & params, const std::string & id);

    void interpretMsr(std::uint16_t msr);
    void interpretMer(std::uint16_t mer);
    void interpretDer(std::uint16_t der);
    void interpretDer2(std::uint16_t der2);
    void interpretCer(std::uint16_t cer);
    void interpretStatusword(std::uint16_t statusword);

    void handleTpdo1(std::uint16_t statusword, std::uint16_t msr, std::int8_t modesOfOperation);
    void handleTpdo2(std::uint16_t mer, std::uint16_t der);
    void handleTpdo3(std::int32_t position, std::int16_t current);
    void handleEmcy(EmcyConsumer::code_t code, std::uint8_t reg, const std::uint8_t * msef);
    void handleNmt(NmtState state);

    bool monitorWorker(const yarp::os::YarpTimerEvent & event);

    bool setPosLimitRaw(double limit, bool isMin);
    bool getPosLimitRaw(double * limit, bool isMin);

    yarp::dev::PolyDriver externalEncoderDevice;
    yarp::dev::IEncodersTimedRaw * iEncodersTimedRawExternal {nullptr};
    roboticslab::ICanBusSharer * iExternalEncoderCanBusSharer {nullptr};

    yarp::os::Timer * monitorThread {nullptr};
    roboticslab::ICanSenderDelegate * sender {nullptr};
};

} // namespace roboticslab

#endif // __TECHNOSOFT_IPOS_BASE_HPP__
