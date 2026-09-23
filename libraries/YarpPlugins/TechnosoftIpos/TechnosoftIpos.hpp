// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __TECHNOSOFT_IPOS_HPP__
#define __TECHNOSOFT_IPOS_HPP__

#include "TechnosoftIposBase.hpp"

#include "TechnosoftIpos_ParamsParser.h"

/**
 * @ingroup YarpPlugins
 * @defgroup TechnosoftIpos
 * @brief Contains TechnosoftIpos.
 */

/**
 * @ingroup TechnosoftIpos
 * @brief Implementation for the Technosoft iPOS as a single CAN bus joint (control board raw interfaces).
 */
class TechnosoftIpos : public yarp::dev::DeviceDriver,
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
                       public roboticslab::ICanBusSharer,
                       public TechnosoftIpos_ParamsParser
{
public:

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    using return_t = yarp::dev::ReturnValue;
#else
    using return_t = bool;
#endif

    //  --------- DeviceDriver declarations. Implementation in DeviceDriverImpl.cpp ---------

    bool open(yarp::os::Searchable & config) override;
    bool close() override;

    std::string id() const override
    { return impl->id(); }

    //  --------- ICanBusSharer declarations. Implementation in ICanBusSharerImpl.cpp ---------

    unsigned int getId() override
    { return impl->getId(); }

    std::vector<unsigned int> getAdditionalIds() override
    { return impl->getAdditionalIds(); }

    bool notifyMessage(const roboticslab::can_message & message) override
    { return impl->notifyMessage(message); }

    bool initialize() override
    { return impl->initialize(); }

    bool finalize() override
    { return impl->finalize(); }

    bool registerSender(roboticslab::ICanSenderDelegate * sender) override
    { return impl->registerSender(sender); }

    bool synchronize(double timestamp) override
    { return impl->synchronize(timestamp); }

    //  --------- IAxisInfoRaw declarations. Implementation in IAxisInfoRawImpl.cpp ---------

    return_t getAxisNameRaw(int axis, std::string & name) override
    { return impl->getAxisNameRaw(axis, name); }

    return_t getJointTypeRaw(int axis, yarp::dev::JointTypeEnum & type) override
    { return impl->getJointTypeRaw(axis, type); }

    //  --------- IControlLimitsRaw declarations. Implementation in IControlLimitsRawImpl.cpp ---------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return_t setPosLimitsRaw(int axis, double min, double max) override
    { return impl->setPosLimitsRaw(axis, min, max); }

    return_t getPosLimitsRaw(int axis, double * min, double * max) override
    { return impl->getPosLimitsRaw(axis, min, max); }
#else
    return_t setLimitsRaw(int axis, double min, double max) override
    { return impl->setLimitsRaw(axis, min, max); }

    return_t getLimitsRaw(int axis, double * min, double * max) override
    { return impl->getLimitsRaw(axis, min, max); }
#endif

    return_t setVelLimitsRaw(int axis, double min, double max) override
    { return impl->setVelLimitsRaw(axis, min, max); }

    return_t getVelLimitsRaw(int axis, double * min, double * max) override
    { return impl->getVelLimitsRaw(axis, min, max); }

    //  --------- IControlModeRaw declarations. Implementation in IControlModeRawImpl.cpp ---------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return_t getAvailableControlModesRaw(int j, std::vector<yarp::dev::SelectableControlModeEnum> & avail) override
    { return impl->getAvailableControlModesRaw(j, avail); }

    return_t getControlModeRaw(int j, yarp::dev::ControlModeEnum & mode) override
    { return impl->getControlModeRaw(j, mode); }

    return_t getControlModesRaw(std::vector<yarp::dev::ControlModeEnum> & modes) override
    { return impl->getControlModesRaw(modes); }

    return_t getControlModesRaw(const std::vector<int> & joints, std::vector<yarp::dev::ControlModeEnum> & modes) override
    { return impl->getControlModesRaw(joints, modes); }

    return_t setControlModeRaw(int j, yarp::dev::SelectableControlModeEnum mode) override
    { return impl->setControlModeRaw(j, mode); }

    return_t setControlModesRaw(const std::vector<yarp::dev::SelectableControlModeEnum> & modes) override
    { return impl->setControlModesRaw(modes); }

    return_t setControlModesRaw(const std::vector<int> & joints, const std::vector<yarp::dev::SelectableControlModeEnum> & modes) override
    { return impl->setControlModesRaw(joints, modes); }
#else
    return_t getControlModeRaw(int j, int * mode) override
    { return impl->getControlModeRaw(j, mode); }

    return_t getControlModesRaw(int * modes) override
    { return impl->getControlModesRaw(modes); }

    return_t getControlModesRaw(int n_joint, const int * joints, int * modes) override
    { return impl->getControlModesRaw(n_joint, joints, modes); }

    return_t setControlModeRaw(int j, int mode) override
    { return impl->setControlModeRaw(j, mode); }

    return_t setControlModesRaw(int * modes) override
    { return impl->setControlModesRaw(modes); }

    return_t setControlModesRaw(int n_joint, const int * joints, int * modes) override
    { return impl->setControlModesRaw(n_joint, joints, modes); }
#endif

    //  --------- ICurrentControlRaw declarations. Implementation in ICurrentControlRawImpl.cpp ---------

    //bool getNumberOfMotorsRaw(int * number) override;

    return_t getCurrentRaw(int m, double * curr) override
    { return impl->getCurrentRaw(m, curr); }

    return_t getCurrentsRaw(double * currs) override
    { return impl->getCurrentsRaw(currs); }

    return_t getCurrentRangeRaw(int m, double * min, double * max) override
    { return impl->getCurrentRangeRaw(m, min, max); }

    return_t getCurrentRangesRaw(double * min, double * max) override
    { return impl->getCurrentRangesRaw(min, max); }

    return_t setRefCurrentRaw(int m, double curr) override
    { return impl->setRefCurrentRaw(m, curr); }

    return_t setRefCurrentsRaw(const double * currs) override
    { return impl->setRefCurrentsRaw(currs); }

    return_t setRefCurrentsRaw(int n_motor, const int * motors, const double * currs) override
    { return impl->setRefCurrentsRaw(n_motor, motors, currs); }

    return_t getRefCurrentRaw(int m, double * curr) override
    { return impl->getRefCurrentRaw(m, curr); }

    return_t getRefCurrentsRaw(double * currs) override
    { return impl->getRefCurrentsRaw(currs); }

    //  ---------- IEncodersRaw declarations. Implementation in IEncodersRawImpl.cpp ----------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return_t getAxes(std::size_t & ax) override
    { return impl->getAxes(ax); }
#else
    return_t getAxes(int * ax) override
    { return impl->getAxes(ax); }
#endif

    return_t resetEncoderRaw(int j) override
    { return impl->resetEncoderRaw(j); }

    return_t resetEncodersRaw() override
    { return impl->resetEncodersRaw(); }

    return_t setEncoderRaw(int j, double val) override
    { return impl->setEncoderRaw(j, val); }

    return_t setEncodersRaw(const double * vals) override
    { return impl->setEncodersRaw(vals); }

    return_t getEncoderRaw(int j, double * v) override
    { return impl->getEncoderRaw(j, v); }

    return_t getEncodersRaw(double * encs) override
    { return impl->getEncodersRaw(encs); }

    return_t getEncoderSpeedRaw(int j, double * sp) override
    { return impl->getEncoderSpeedRaw(j, sp); }

    return_t getEncoderSpeedsRaw(double * spds) override
    { return impl->getEncoderSpeedsRaw(spds); }

    return_t getEncoderAccelerationRaw(int j, double * spds) override
    { return impl->getEncoderAccelerationRaw(j, spds); }

    return_t getEncoderAccelerationsRaw(double * accs) override
    { return impl->getEncoderAccelerationsRaw(accs); }

    //  ---------- IEncodersTimedRaw declarations. Implementation in IEncodersRawImpl.cpp ----------

    return_t getEncoderTimedRaw(int j, double * encs, double * time) override
    { return impl->getEncoderTimedRaw(j, encs, time); }

    return_t getEncodersTimedRaw(double * encs, double * time) override
    { return impl->getEncodersTimedRaw(encs, time); }

    //  ---------- IImpedanceControlRaw declarations. Implementation in IImpedanceControlRawImpl.cpp ----------

    //bool getAxes(int * ax) override;

    return_t getImpedanceRaw(int j, double * stiffness, double * damping) override
    { return impl->getImpedanceRaw(j, stiffness, damping); }

    return_t setImpedanceRaw(int j, double stiffness, double damping) override
    { return impl->setImpedanceRaw(j, stiffness, damping); }

    return_t setImpedanceOffsetRaw(int j, double offset) override
    { return impl->setImpedanceOffsetRaw(j, offset); }

    return_t getImpedanceOffsetRaw(int j, double * offset) override
    { return impl->getImpedanceOffsetRaw(j, offset); }

    return_t getCurrentImpedanceLimitRaw(int j, double * min_stiff, double * max_stiff, double * min_damp, double * max_damp) override
    { return impl->getCurrentImpedanceLimitRaw(j, min_stiff, max_stiff, min_damp, max_damp); }

    //  ---------- IInteractionModeRaw declarations. Implementation in IInteractionModeRawImpl.cpp ----------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return_t getInteractionModeRaw(int axis, yarp::dev::InteractionModeEnum & mode) override
    { return impl->getInteractionModeRaw(axis, mode); }

    return_t getInteractionModesRaw(std::vector<yarp::dev::InteractionModeEnum> & modes) override
    { return impl->getInteractionModesRaw(modes); }

    return_t getInteractionModesRaw(const std::vector<int> & joints, std::vector<yarp::dev::InteractionModeEnum> & modes) override
    { return impl->getInteractionModesRaw(joints, modes); }

    return_t setInteractionModeRaw(int axis, yarp::dev::InteractionModeEnum mode) override
    { return impl->setInteractionModeRaw(axis, mode); }

    return_t setInteractionModesRaw(const std::vector<yarp::dev::InteractionModeEnum> & modes) override
    { return impl->setInteractionModesRaw(modes); }

    return_t setInteractionModesRaw(const std::vector<int> & joints, const std::vector<yarp::dev::InteractionModeEnum> & modes) override
    { return impl->setInteractionModesRaw(joints, modes); }
#else
    return_t getInteractionModeRaw(int axis, yarp::dev::InteractionModeEnum * mode) override
    { return impl->getInteractionModeRaw(axis, mode); }

    return_t getInteractionModesRaw(int n_joints, int * joints, yarp::dev::InteractionModeEnum * modes) override
    { return impl->getInteractionModesRaw(n_joints, joints, modes); }

    return_t getInteractionModesRaw(yarp::dev::InteractionModeEnum * modes) override
    { return impl->getInteractionModesRaw(modes); }

    return_t setInteractionModeRaw(int axis, yarp::dev::InteractionModeEnum mode) override
    { return impl->setInteractionModeRaw(axis, mode); }

    return_t setInteractionModesRaw(int n_joints, int * joints, yarp::dev::InteractionModeEnum * modes) override
    { return impl->setInteractionModesRaw(n_joints, joints, modes); }

    return_t setInteractionModesRaw(yarp::dev::InteractionModeEnum * modes) override
    { return impl->setInteractionModesRaw(modes); }
#endif

    //  ---------- IJointFaultRaw declarations. Implementation in IJointFaultRawImpl.cpp ----------

    return_t getLastJointFaultRaw(int j, int & fault, std::string & message) override
    { return impl->getLastJointFaultRaw(j, fault, message); }

    //  --------- IMotorRaw declarations. Implementation in IMotorRawImpl.cpp ---------

    return_t getNumberOfMotorsRaw(int * num) override
    { return impl->getNumberOfMotorsRaw(num); }

    return_t getTemperatureRaw(int m, double * val) override
    { return impl->getTemperatureRaw(m, val); }

    return_t getTemperaturesRaw(double * vals) override
    { return impl->getTemperaturesRaw(vals); }

    return_t getTemperatureLimitRaw(int m, double * temp) override
    { return impl->getTemperatureLimitRaw(m, temp); }

    return_t setTemperatureLimitRaw(int m, double temp) override
    { return impl->setTemperatureLimitRaw(m, temp); }

    return_t getGearboxRatioRaw(int m, double * val) override
    { return impl->getGearboxRatioRaw(m, val); }

    return_t setGearboxRatioRaw(int m, double val) override
    { return impl->setGearboxRatioRaw(m, val); }

    //  --------- IMotorEncodersRaw declarations. Implementation in IMotorEncodersRawImpl.cpp ---------

    return_t getNumberOfMotorEncodersRaw(int * num) override
    { return impl->getNumberOfMotorEncodersRaw(num); }

    return_t resetMotorEncoderRaw(int m) override
    { return impl->resetMotorEncoderRaw(m); }

    return_t resetMotorEncodersRaw() override
    { return impl->resetMotorEncodersRaw(); }

    return_t setMotorEncoderCountsPerRevolutionRaw(int m, double cpr) override
    { return impl->setMotorEncoderCountsPerRevolutionRaw(m, cpr); }

    return_t getMotorEncoderCountsPerRevolutionRaw(int m, double * cpr) override
    { return impl->getMotorEncoderCountsPerRevolutionRaw(m, cpr); }

    return_t setMotorEncoderRaw(int m, double val) override
    { return impl->setMotorEncoderRaw(m, val); }

    return_t setMotorEncodersRaw(const double * vals) override
    { return impl->setMotorEncodersRaw(vals); }

    return_t getMotorEncoderRaw(int m, double * v) override
    { return impl->getMotorEncoderRaw(m, v); }

    return_t getMotorEncodersRaw(double * encs) override
    { return impl->getMotorEncodersRaw(encs); }

    return_t getMotorEncoderTimedRaw(int m, double * encs, double * stamp) override
    { return impl->getMotorEncoderTimedRaw(m, encs, stamp); }

    return_t getMotorEncodersTimedRaw(double * encs, double * stamps) override
    { return impl->getMotorEncodersTimedRaw(encs, stamps); }

    return_t getMotorEncoderSpeedRaw(int m, double * sp) override
    { return impl->getMotorEncoderSpeedRaw(m, sp); }

    return_t getMotorEncoderSpeedsRaw(double * spds) override
    { return impl->getMotorEncoderSpeedsRaw(spds); }

    return_t getMotorEncoderAccelerationRaw(int m, double * spds) override
    { return impl->getMotorEncoderAccelerationRaw(m, spds); }

    return_t getMotorEncoderAccelerationsRaw(double * vaccs) override
    { return impl->getMotorEncoderAccelerationsRaw(vaccs); }

    //  --------- IPidControlRaw declarations. Implementation in IPidControlRawImpl.cpp ---------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return_t getAvailablePidsRaw(int j, std::vector<yarp::dev::PidControlTypeEnum> & avail) override
    { return impl->getAvailablePidsRaw(j, avail); }
#endif

    return_t setPidRaw(const yarp::dev::PidControlTypeEnum & pidtype, int j, const yarp::dev::Pid & pid) override
    { return impl->setPidRaw(pidtype, j, pid); }

    return_t setPidsRaw(const yarp::dev::PidControlTypeEnum & pidtype, const yarp::dev::Pid * pids) override
    { return impl->setPidsRaw(pidtype, pids); }

    return_t setPidReferenceRaw(const yarp::dev::PidControlTypeEnum & pidtype, int j, double ref) override
    { return impl->setPidReferenceRaw(pidtype, j, ref); }

    return_t setPidReferencesRaw(const yarp::dev::PidControlTypeEnum & pidtype, const double * refs) override
    { return impl->setPidReferencesRaw(pidtype, refs); }

    return_t setPidErrorLimitRaw(const yarp::dev::PidControlTypeEnum & pidtype, int j, double limit) override
    { return impl->setPidErrorLimitRaw(pidtype, j, limit); }

    return_t setPidErrorLimitsRaw(const yarp::dev::PidControlTypeEnum & pidtype, const double * limits) override
    { return impl->setPidErrorLimitsRaw(pidtype, limits); }

    return_t getPidErrorRaw(const yarp::dev::PidControlTypeEnum & pidtype, int j, double * err) override
    { return impl->getPidErrorRaw(pidtype, j, err); }

    return_t getPidErrorsRaw(const yarp::dev::PidControlTypeEnum & pidtype, double * errs) override
    { return impl->getPidErrorsRaw(pidtype, errs); }

    return_t getPidOutputRaw(const yarp::dev::PidControlTypeEnum & pidtype, int j, double * out) override
    { return impl->getPidOutputRaw(pidtype, j, out); }

    return_t getPidOutputsRaw(const yarp::dev::PidControlTypeEnum & pidtype, double * outs) override
    { return impl->getPidOutputsRaw(pidtype, outs); }

    return_t getPidRaw(const yarp::dev::PidControlTypeEnum & pidtype, int j, yarp::dev::Pid * pid) override
    { return impl->getPidRaw(pidtype, j, pid); }

    return_t getPidsRaw(const yarp::dev::PidControlTypeEnum & pidtype, yarp::dev::Pid * pids) override
    { return impl->getPidsRaw(pidtype, pids); }

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return_t getPidOffsetRaw(const yarp::dev::PidControlTypeEnum & pidtype, int j, double & v) override
    { return impl->getPidOffsetRaw(pidtype, j, v); }

    return_t getPidFeedforwardRaw(const yarp::dev::PidControlTypeEnum & pidtype, int j, double & v) override
    { return impl->getPidFeedforwardRaw(pidtype, j, v); }

    return_t getPidExtraInfoRaw(const yarp::dev::PidControlTypeEnum & pidtype, int j, yarp::dev::PidExtraInfo & info) override
    { return impl->getPidExtraInfoRaw(pidtype, j, info); }

    return_t getPidExtraInfosRaw(const yarp::dev::PidControlTypeEnum & pidtype, std::vector<yarp::dev::PidExtraInfo> & info) override
    { return impl->getPidExtraInfosRaw(pidtype, info); }
#endif

    return_t getPidReferenceRaw(const yarp::dev::PidControlTypeEnum & pidtype, int j, double * ref) override
    { return impl->getPidReferenceRaw(pidtype, j, ref); }

    return_t getPidReferencesRaw(const yarp::dev::PidControlTypeEnum & pidtype, double * refs) override
    { return impl->getPidReferencesRaw(pidtype, refs); }

    return_t getPidErrorLimitRaw(const yarp::dev::PidControlTypeEnum & pidtype, int j, double * limit) override
    { return impl->getPidErrorLimitRaw(pidtype, j, limit); }

    return_t getPidErrorLimitsRaw(const yarp::dev::PidControlTypeEnum & pidtype, double * limits) override
    { return impl->getPidErrorLimitsRaw(pidtype, limits); }

    return_t resetPidRaw(const yarp::dev::PidControlTypeEnum & pidtype, int j) override
    { return impl->resetPidRaw(pidtype, j); }

    return_t disablePidRaw(const yarp::dev::PidControlTypeEnum & pidtype, int j) override
    { return impl->disablePidRaw(pidtype, j); }

    return_t enablePidRaw(const yarp::dev::PidControlTypeEnum & pidtype, int j) override
    { return impl->enablePidRaw(pidtype, j); }

    return_t setPidOffsetRaw(const yarp::dev::PidControlTypeEnum & pidtype, int j, double v) override
    { return impl->setPidOffsetRaw(pidtype, j, v); }

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return_t setPidFeedforwardRaw(const yarp::dev::PidControlTypeEnum & pidtype, int j, double v) override
    { return impl->setPidFeedforwardRaw(pidtype, j, v); }

    return_t isPidEnabledRaw(const yarp::dev::PidControlTypeEnum & pidtype, int j, bool & enabled) override
    { return impl->isPidEnabledRaw(pidtype, j, enabled); }
#else
    return_t isPidEnabledRaw(const yarp::dev::PidControlTypeEnum & pidtype, int j, bool * enabled) override
    { return impl->isPidEnabledRaw(pidtype, j, enabled); }
#endif

    // ------- IPositionControlRaw declarations. Implementation in IPositionControlRawImpl.cpp -------

    return_t positionMoveRaw(int j, double ref) override
    { return impl->positionMoveRaw(j, ref); }

    return_t positionMoveRaw(const double * refs) override
    { return impl->positionMoveRaw(refs); }

    return_t positionMoveRaw(int n_joint, const int * joints, const double * refs) override
    { return impl->positionMoveRaw(n_joint, joints, refs); }

    return_t relativeMoveRaw(int j, double delta) override
    { return impl->relativeMoveRaw(j, delta); }

    return_t relativeMoveRaw(const double * deltas) override
    { return impl->relativeMoveRaw(deltas); }

    return_t relativeMoveRaw(int n_joint, const int * joints, const double * deltas) override
    { return impl->relativeMoveRaw(n_joint, joints, deltas); }

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return_t checkMotionDoneRaw(int j, bool & flag) override
    { return impl->checkMotionDoneRaw(j, flag); }

    return_t checkMotionDoneRaw(bool & flag) override
    { return impl->checkMotionDoneRaw(flag); }

    return_t checkMotionDoneRaw(const std::vector<int> & joints, bool & flag) override
    { return impl->checkMotionDoneRaw(joints, flag); }

    return_t setTrajSpeedRaw(int j, double sp) override
    { return impl->setTrajSpeedRaw(j, sp); }

    return_t setTrajSpeedsRaw(const double * spds) override
    { return impl->setTrajSpeedsRaw(spds); }

    return_t setTrajSpeedsRaw(int n_joint, const int * joints, const double * spds) override
    { return impl->setTrajSpeedsRaw(n_joint, joints, spds); }

    return_t setTrajAccelerationRaw(int j, double acc) override
    { return impl->setTrajAccelerationRaw(j, acc); }

    return_t setTrajAccelerationsRaw(const double * accs) override
    { return impl->setTrajAccelerationsRaw(accs); }

    return_t setTrajAccelerationsRaw(int n_joint, const int * joints, const double * accs) override
    { return impl->setTrajAccelerationsRaw(n_joint, joints, accs); }

    return_t getTrajSpeedRaw(int j, double * spds) override
    { return impl->getTrajSpeedRaw(j, spds); }

    return_t getTrajSpeedsRaw(double * spds) override
    { return impl->getTrajSpeedsRaw(spds); }

    return_t getTrajSpeedsRaw(int n_joint, const int * joints, double * spds) override
    { return impl->getTrajSpeedsRaw(n_joint, joints, spds); }

    return_t getTrajAccelerationRaw(int j, double * acc) override
    { return impl->getTrajAccelerationRaw(j, acc); }

    return_t getTrajAccelerationsRaw(double * accs) override
    { return impl->getTrajAccelerationsRaw(accs); }

    return_t getTrajAccelerationsRaw(int n_joint, const int * joints, double * accs) override
    { return impl->getTrajAccelerationsRaw(n_joint, joints, accs); }
#else
    return_t checkMotionDoneRaw(int j, bool * flag) override
    { return impl->checkMotionDoneRaw(j, flag); }

    return_t checkMotionDoneRaw(bool * flag) override
    { return impl->checkMotionDoneRaw(flag); }

    return_t checkMotionDoneRaw(int n_joint, const int * joints, bool * flag) override
    { return impl->checkMotionDoneRaw(n_joint, joints, flag); }

    return_t setRefSpeedRaw(int j, double sp) override
    { return impl->setRefSpeedRaw(j, sp); }

    return_t setRefSpeedsRaw(const double * spds) override
    { return impl->setRefSpeedsRaw(spds); }

    return_t setRefSpeedsRaw(int n_joint, const int * joints, const double * spds) override
    { return impl->setRefSpeedsRaw(n_joint, joints, spds); }

    return_t setRefAccelerationRaw(int j, double acc) override
    { return impl->setRefAccelerationRaw(j, acc); }

    return_t setRefAccelerationsRaw(const double * accs) override
    { return impl->setRefAccelerationsRaw(accs); }

    return_t setRefAccelerationsRaw(int n_joint, const int * joints, const double * accs) override
    { return impl->setRefAccelerationsRaw(n_joint, joints, accs); }

    return_t getRefSpeedRaw(int j, double * ref) override
    { return impl->getRefSpeedRaw(j, ref); }

    return_t getRefSpeedsRaw(double * spds) override
    { return impl->getRefSpeedsRaw(spds); }

    return_t getRefSpeedsRaw(int n_joint, const int * joints, double * spds) override
    { return impl->getRefSpeedsRaw(n_joint, joints, spds); }

    return_t getRefAccelerationRaw(int j, double * acc) override
    { return impl->getRefAccelerationRaw(j, acc); }

    return_t getRefAccelerationsRaw(double * accs) override
    { return impl->getRefAccelerationsRaw(accs); }

    return_t getRefAccelerationsRaw(int n_joint, const int * joints, double * accs) override
    { return impl->getRefAccelerationsRaw(n_joint, joints, accs); }
#endif

    return_t stopRaw(int j) override
    { return impl->stopRaw(j); }

    return_t stopRaw() override
    { return impl->stopRaw(); }

    return_t stopRaw(int n_joint, const int * joints) override
    { return impl->stopRaw(n_joint, joints); }

    return_t getTargetPositionRaw(int joint, double * ref) override
    { return impl->getTargetPositionRaw(joint, ref); }

    return_t getTargetPositionsRaw(double * refs) override
    { return impl->getTargetPositionsRaw(refs); }

    return_t getTargetPositionsRaw(int n_joint, const int * joints, double * refs) override
    { return impl->getTargetPositionsRaw(n_joint, joints, refs); }

    // ------- IPositionDirectRaw declarations. Implementation in IPositionDirectRawImpl.cpp -------

    return_t setPositionRaw(int j, double ref) override
    { return impl->setPositionRaw(j, ref); }

    return_t setPositionsRaw(const double * refs) override
    { return impl->setPositionsRaw(refs); }

    return_t setPositionsRaw(int n_joint, const int * joints, const double * refs) override
    { return impl->setPositionsRaw(n_joint, joints, refs); }

    return_t getRefPositionRaw(int joint, double * ref) override
    { return impl->getRefPositionRaw(joint, ref); }

    return_t getRefPositionsRaw(double * refs) override
    { return impl->getRefPositionsRaw(refs); }

    return_t getRefPositionsRaw(int n_joint, const int * joints, double * refs) override
    { return impl->getRefPositionsRaw(n_joint, joints, refs); }

    // ------- IRemoteVariablesRaw declarations. Implementation in IRemoteVariablesRawImpl.cpp -------

    return_t getRemoteVariableRaw(std::string key, yarp::os::Bottle & val) override
    { return impl->getRemoteVariableRaw(key, val); }

    return_t setRemoteVariableRaw(std::string key, const yarp::os::Bottle & val) override
    { return impl->setRemoteVariableRaw(key, val); }

    return_t getRemoteVariablesListRaw(yarp::os::Bottle * listOfKeys) override
    { return impl->getRemoteVariablesListRaw(listOfKeys); }

    // -------- ITorqueControlRaw declarations. Implementation in ITorqueControlRawImpl.cpp --------

    return_t getRefTorqueRaw(int j, double * t) override
    { return impl->getRefTorqueRaw(j, t); }

    return_t getRefTorquesRaw(double * t) override
    { return impl->getRefTorquesRaw(t); }

    return_t setRefTorqueRaw(int j, double t) override
    { return impl->setRefTorqueRaw(j, t); }

    return_t setRefTorquesRaw(int n_joint, const int * joints, const double * t) override
    { return impl->setRefTorquesRaw(n_joint, joints, t); }

    return_t setRefTorquesRaw(const double * t) override
    { return impl->setRefTorquesRaw(t); }

    return_t getTorqueRaw(int j, double * t) override
    { return impl->getTorqueRaw(j, t); }

    return_t getTorquesRaw(double * t) override
    { return impl->getTorquesRaw(t); }

    return_t getTorqueRangeRaw(int j, double * min, double * max) override
    { return impl->getTorqueRangeRaw(j, min, max); }

    return_t getTorqueRangesRaw(double * min, double * max) override
    { return impl->getTorqueRangesRaw(min, max); }

    return_t getMotorTorqueParamsRaw(int j, yarp::dev::MotorTorqueParameters * params) override
    { return impl->getMotorTorqueParamsRaw(j, params); }

    return_t setMotorTorqueParamsRaw(int j, const yarp::dev::MotorTorqueParameters params) override
    { return impl->setMotorTorqueParamsRaw(j, params); }

    //  --------- IVelocityControlRaw declarations. Implementation in IVelocityControlRawImpl.cpp ---------

    return_t velocityMoveRaw(int j, double sp) override
    { return impl->velocityMoveRaw(j, sp); }

    return_t velocityMoveRaw(const double * sp) override
    { return impl->velocityMoveRaw(sp); }

    return_t velocityMoveRaw(int n_joint, const int * joints, const double * spds) override
    { return impl->velocityMoveRaw(n_joint, joints, spds); }

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return_t getTargetVelocityRaw(int joint, double * vel) override
    { return impl->getTargetVelocityRaw(joint, vel); }

    return_t getTargetVelocitiesRaw(double * vels) override
    { return impl->getTargetVelocitiesRaw(vels); }

    return_t getTargetVelocitiesRaw(int n_joint, const int * joints, double * vels) override
    { return impl->getTargetVelocitiesRaw(n_joint, joints, vels); }
#else
    return_t getRefVelocityRaw(int joint, double * vel) override
    { return impl->getRefVelocityRaw(joint, vel); }

    return_t getRefVelocitiesRaw(double * vels) override
    { return impl->getRefVelocitiesRaw(vels); }

    return_t getRefVelocitiesRaw(int n_joint, const int * joints, double * vels) override
    { return impl->getRefVelocitiesRaw(n_joint, joints, vels); }
#endif

private:

    roboticslab::TechnosoftIposBase * impl {nullptr};
};

#endif // __TECHNOSOFT_IPOS_HPP__
