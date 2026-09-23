// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __CAN_BUS_BROKER_HPP__
#define __CAN_BUS_BROKER_HPP__

#include <vector>

#include <yarp/conf/version.h>

#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/MultipleAnalogSensorsInterfaces.h>

#include "DeviceMapper.hpp"
#include "SingleBusBroker.hpp"
#include "SyncPeriodicThread.hpp"
#include "CanBusBroker_ParamsParser.h"

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
#define CHECK_JOINT(j) do { int n = deviceMapper.getControlledAxes(); if ((j) < 0 || (j) > n - 1) return yarp::dev::ReturnValue_error_input_out_of_bounds; } while (0)
#else
#define CHECK_JOINT(j) do { int n = deviceMapper.getControlledAxes(); if ((j) < 0 || (j) > n - 1) return false; } while (0)
#endif

/**
 * @ingroup YarpPlugins
 * @defgroup CanBusBroker
 * @brief Contains CanBusBroker.
 */

/**
 * @ingroup CanBusBroker
 * @brief CAN-oriented control board that implements all YARP motor interfaces.
 *
 * This control board wrapper subdevice exposes motor commands to CAN nodes
 * modelled as wrapped motor raw subdevices (i.e. devices which implement the
 * xxxRaw interface counterparts). At the core of this driver class, another set
 * of wrapped devices implements hardware connection to a physical CAN bus, thus
 * allowing CAN reads and writes that CanBusBroker manages asynchronously with
 * regard to exposed YARP commands (see @ref CanReaderWriterThread).
 *
 * This device also supports fake CAN buses and fake CAN nodes, see
 * <a href="https://github.com/roboticslab-uc3m/yarp-devices/issues/241#issuecomment-569112698">instructions</a>.
 */
class CanBusBroker : public yarp::dev::DeviceDriver,
                     public CanBusBroker_ParamsParser,
                     // control board interfaces
                     public yarp::dev::IAmplifierControl,
                     public yarp::dev::IAxisInfo,
                     public yarp::dev::IControlCalibration,
                     public yarp::dev::IControlLimits,
                     public yarp::dev::IControlMode,
                     public yarp::dev::ICurrentControl,
                     public yarp::dev::IEncodersTimed,
                     public yarp::dev::IImpedanceControl,
                     public yarp::dev::IInteractionMode,
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
                     public yarp::dev::IJointBrake,
#endif
                     public yarp::dev::IJointFault,
                     public yarp::dev::IMotor,
                     public yarp::dev::IMotorEncoders,
                     public yarp::dev::IPidControl,
                     public yarp::dev::IPositionControl,
                     public yarp::dev::IPositionDirect,
                     public yarp::dev::IPWMControl,
                     public yarp::dev::IRemoteVariables,
                     public yarp::dev::ITorqueControl,
                     public yarp::dev::IVelocityControl,
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
                     public yarp::dev::IVelocityDirect,
#endif
                     // multiple analog sensors interfaces
                     public yarp::dev::IContactLoadCellArrays,
                     public yarp::dev::IEncoderArrays,
                     public yarp::dev::IOrientationSensors,
                     public yarp::dev::IPositionSensors,
                     public yarp::dev::ISixAxisForceTorqueSensors,
                     public yarp::dev::ISkinPatches,
                     public yarp::dev::ITemperatureSensors,
                     public yarp::dev::IThreeAxisGyroscopes,
                     public yarp::dev::IThreeAxisLinearAccelerometers,
                     public yarp::dev::IThreeAxisMagnetometers
{
public:
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    using return_t = yarp::dev::ReturnValue;
#else
    using return_t = bool;
#endif

    // -------- DeviceDriver declarations. Implementation in DeviceDriverImpl.cpp --------

    bool open(yarp::os::Searchable & config) override;
    bool close() override;

    // ---------- CONTROL BOARD INTERFACES ----------

    // --------- IAmplifierControl declarations. Implementation in IAmplifierControlImpl.cpp ---------

    return_t enableAmp(int j) override;
    return_t disableAmp(int j) override;
    return_t getAmpStatus(int j, int * v) override;
    return_t getAmpStatus(int * st) override;
    //return_t getCurrent(int j, double * val) override;
    //return_t getCurrents(double * vals) override;
    return_t getMaxCurrent(int j, double * v) override;
    return_t setMaxCurrent(int j, double v) override;
    return_t getNominalCurrent(int m, double * val) override;
    return_t setNominalCurrent(int m, double val) override;
    return_t getPeakCurrent(int m, double * val) override;
    return_t setPeakCurrent(int m, double val) override;
    return_t getPWM(int j, double * val) override;
    return_t getPWMLimit(int j, double * val) override;
    return_t setPWMLimit(int j, double val) override;
    return_t getPowerSupplyVoltage(int j, double * val) override;

    // --------- IAxisInfo declarations. Implementation in IAxisInfoImpl.cpp ---------

    return_t getAxisName(int axis, std::string & name) override;
    return_t getJointType(int axis, yarp::dev::JointTypeEnum & type) override;

    // --------- IControlCalibration declarations. Implementation in IControlCalibrationImpl.cpp ---------

    return_t calibrateAxisWithParams(int axis, unsigned int type, double p1, double p2, double p3) override;
    return_t setCalibrationParameters(int axis, const yarp::dev::CalibrationParameters & params) override;
    return_t calibrationDone(int j) override;

    // --------- IControlLimits declarations. Implementation in IControlLimitsImpl.cpp ---------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return_t setPosLimits(int axis, double min, double max) override;
    return_t getPosLimits(int axis, double * min, double * max) override;
#else
    return_t setLimits(int axis, double min, double max) override;
    return_t getLimits(int axis, double * min, double * max) override;
#endif
    return_t setVelLimits(int axis, double min, double max) override;
    return_t getVelLimits(int axis, double * min, double * max) override;

    // --------- IControlMode declarations. Implementation in IControlModeImpl.cpp ---------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return_t getAvailableControlModes(int j, std::vector<yarp::dev::SelectableControlModeEnum> & avail) override;
    return_t getControlMode(int j, yarp::dev::ControlModeEnum & mode) override;
    return_t getControlModes(std::vector<yarp::dev::ControlModeEnum> & modes) override;
    return_t getControlModes(const std::vector<int> & joints, std::vector<yarp::dev::ControlModeEnum> & modes) override;
    return_t setControlMode(int j, yarp::dev::SelectableControlModeEnum mode) override;
    return_t setControlModes(const std::vector<int> & joints, const std::vector<yarp::dev::SelectableControlModeEnum> & modes) override;
    return_t setControlModes(const std::vector<yarp::dev::SelectableControlModeEnum> & modes) override;
#else
    return_t getControlMode(int j, int * mode) override;
    return_t getControlModes(int * modes) override;
    return_t getControlModes(int n_joint, const int * joints, int * modes) override;
    return_t setControlMode(int j, const int mode) override;
    return_t setControlModes(int n_joint, const int * joints, int * modes) override;
    return_t setControlModes(int * modes) override;
#endif

    // --------- ICurrentControl declarations. Implementation in ICurrentControlImpl.cpp ---------

    //return_t getNumberOfMotors(int * ax) override;
    return_t getCurrent(int m, double * curr) override;
    return_t getCurrents(double * currs) override;
    return_t getCurrentRange(int m, double * min, double * max) override;
    return_t getCurrentRanges(double * mins, double * maxs) override;
    return_t setRefCurrent(int m, double curr) override;
    return_t setRefCurrents(const double * currs) override;
    return_t setRefCurrents(int n_motor, const int * motors, const double * currs) override;
    return_t getRefCurrent(int m, double * curr) override;
    return_t getRefCurrents(double * currs) override;

    // ---------- IEncoders declarations. Implementation in IEncodersImpl.cpp ----------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return_t getAxes(std::size_t & ax) override;
#else
    return_t getAxes(int * ax) override;
#endif
    return_t resetEncoder(int j) override;
    return_t resetEncoders() override;
    return_t setEncoder(int j, double val) override;
    return_t setEncoders(const double * vals) override;
    return_t getEncoder(int j, double * v) override;
    return_t getEncoders(double * encs) override;
    return_t getEncoderSpeed(int j, double * spd) override;
    return_t getEncoderSpeeds(double * spds) override;
    return_t getEncoderAcceleration(int j, double * spds) override;
    return_t getEncoderAccelerations(double * accs) override;

    // ---------- IEncodersTimed declarations. Implementation in IEncodersImpl.cpp ----------

    return_t getEncoderTimed(int j, double * encs, double * time) override;
    return_t getEncodersTimed(double * encs, double * times) override;

    // --------- IImpedanceControl declarations. Implementation in IImpedanceControlImpl.cpp ---------

    //return_t getAxes(int * ax) override;
    return_t getImpedance(int j, double * stiffness, double * damping) override;
    return_t setImpedance(int j, double stiffness, double damping) override;
    return_t setImpedanceOffset(int j, double offset) override;
    return_t getImpedanceOffset(int j, double * offset) override;
    return_t getCurrentImpedanceLimit(int j, double * min_stiff, double * max_stiff, double * min_damp, double * max_damp) override;

    // -----------IInteractionMode declarations. Implementation in IInteractionModeImpl.cpp --------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return_t getInteractionMode(int axis, yarp::dev::InteractionModeEnum & mode) override;
    return_t getInteractionModes(std::vector<yarp::dev::InteractionModeEnum> & modes) override;
    return_t getInteractionModes(const std::vector<int> & joints, std::vector<yarp::dev::InteractionModeEnum> & modes) override;
    return_t setInteractionMode(int axis, yarp::dev::InteractionModeEnum mode) override;
    return_t setInteractionModes(const std::vector<yarp::dev::InteractionModeEnum> & modes) override;
    return_t setInteractionModes(const std::vector<int> & joints, const std::vector<yarp::dev::InteractionModeEnum> & modes) override;
#else
    return_t getInteractionMode(int axis, yarp::dev::InteractionModeEnum * mode) override;
    return_t getInteractionModes(yarp::dev::InteractionModeEnum * modes) override;
    return_t getInteractionModes(int n_joints, int * joints, yarp::dev::InteractionModeEnum * modes) override;
    return_t setInteractionMode(int axis, yarp::dev::InteractionModeEnum mode) override;
    return_t setInteractionModes(yarp::dev::InteractionModeEnum * modes) override;
    return_t setInteractionModes(int n_joints, int * joints, yarp::dev::InteractionModeEnum * modes) override;
#endif

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    // --------- IJointBrake declarations. Implementation in IJointBrakeImpl.cpp ---------

    return_t isJointBraked(int j, bool & braked) const override;
    return_t setManualBrakeActive(int j, bool active) override;
    return_t setAutoBrakeEnabled(int j, bool enabled) override;
    return_t getAutoBrakeEnabled(int j, bool & enabled) const override;
#endif

    // --------- IJointFault declarations. Implementation in IJointFaultImpl.cpp ---------

    return_t getLastJointFault(int j, int & fault, std::string & message) override;

    // --------- IMotor declarations. Implementation in IMotorImpl.cpp ---------

    return_t getNumberOfMotors(int * num) override;
    return_t getTemperature(int m, double * val) override;
    return_t getTemperatures(double * vals) override;
    return_t getTemperatureLimit(int m, double * temp) override;
    return_t setTemperatureLimit(int m, double temp) override;
    return_t getGearboxRatio(int m, double * val) override;
    return_t setGearboxRatio(int m, double val) override;

    // --------- IMotorEncoders declarations. Implementation in IMotorEncodersImpl.cpp ---------

    return_t getNumberOfMotorEncoders(int * num) override;
    return_t resetMotorEncoder(int m) override;
    return_t resetMotorEncoders() override;
    return_t setMotorEncoderCountsPerRevolution(int m, double cpr) override;
    return_t getMotorEncoderCountsPerRevolution(int m, double * cpr) override;
    return_t setMotorEncoder(int m, double val) override;
    return_t setMotorEncoders(const double * vals) override;
    return_t getMotorEncoder(int m, double * v) override;
    return_t getMotorEncoders(double * encs) override;
    return_t getMotorEncoderTimed(int m, double * enc, double * stamp) override;
    return_t getMotorEncodersTimed(double * encs, double * stamps) override;
    return_t getMotorEncoderSpeed(int m, double * sp) override;
    return_t getMotorEncoderSpeeds(double *spds) override;
    return_t getMotorEncoderAcceleration(int m, double * acc) override;
    return_t getMotorEncoderAccelerations(double * accs) override;

    // --------- IPidControl declarations. Implementation in IPidControlImpl.cpp ---------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return_t getAvailablePids(int j, std::vector<yarp::dev::PidControlTypeEnum> & avail) override;
#endif
    return_t setPid(const yarp::dev::PidControlTypeEnum & pidtype, int j, const yarp::dev::Pid & pid) override;
    return_t setPids(const yarp::dev::PidControlTypeEnum & pidtype, const yarp::dev::Pid * pids) override;
    return_t setPidReference(const yarp::dev::PidControlTypeEnum & pidtype, int j, double ref) override;
    return_t setPidReferences(const yarp::dev::PidControlTypeEnum & pidtype, const double * refs) override;
    return_t setPidErrorLimit(const yarp::dev::PidControlTypeEnum & pidtype, int j, double limit) override;
    return_t setPidErrorLimits(const yarp::dev::PidControlTypeEnum & pidtype, const double * limits) override;
    return_t getPidError(const yarp::dev::PidControlTypeEnum & pidtype, int j, double * err) override;
    return_t getPidErrors(const yarp::dev::PidControlTypeEnum & pidtype, double * errs) override;
    return_t getPidOutput(const yarp::dev::PidControlTypeEnum & pidtype, int j, double * out) override;
    return_t getPidOutputs(const yarp::dev::PidControlTypeEnum & pidtype, double * outs) override;
    return_t getPid(const yarp::dev::PidControlTypeEnum & pidtype, int j, yarp::dev::Pid * pid) override;
    return_t getPids(const yarp::dev::PidControlTypeEnum & pidtype, yarp::dev::Pid * pids) override;
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return_t getPidOffset(const yarp::dev::PidControlTypeEnum & pidtype, int j, double & v) override;
    return_t getPidFeedforward(const yarp::dev::PidControlTypeEnum & pidtype, int j, double & v) override;
    return_t getPidExtraInfo(const yarp::dev::PidControlTypeEnum & pidtype, int j, yarp::dev::PidExtraInfo & info) override;
    return_t getPidExtraInfos(const yarp::dev::PidControlTypeEnum & pidtype, std::vector<yarp::dev::PidExtraInfo> & info) override;
#endif
    return_t getPidReference(const yarp::dev::PidControlTypeEnum & pidtype, int j, double * ref) override;
    return_t getPidReferences(const yarp::dev::PidControlTypeEnum & pidtype, double * refs) override;
    return_t getPidErrorLimit(const yarp::dev::PidControlTypeEnum & pidtype, int j, double * limit) override;
    return_t getPidErrorLimits(const yarp::dev::PidControlTypeEnum & pidtype, double * limits) override;
    return_t resetPid(const yarp::dev::PidControlTypeEnum & pidtype, int j) override;
    return_t disablePid(const yarp::dev::PidControlTypeEnum & pidtype, int j) override;
    return_t enablePid(const yarp::dev::PidControlTypeEnum & pidtype, int j) override;
    return_t setPidOffset(const yarp::dev::PidControlTypeEnum & pidtype, int j, double v) override;
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return_t setPidFeedforward(const yarp::dev::PidControlTypeEnum & pidtype, int j, double v) override;
    return_t isPidEnabled(const yarp::dev::PidControlTypeEnum & pidtype, int j, bool & enabled) override;
#else
    return_t isPidEnabled(const yarp::dev::PidControlTypeEnum & pidtype, int j, bool * enabled) override;
#endif

    // ------- IPositionControl declarations. Implementation in IPositionControlImpl.cpp -------

    //return_t getAxes(std::size_t & ax) override;
    return_t positionMove(int j, double ref) override;
    return_t positionMove(const double * refs) override;
    return_t positionMove(int n_joint, const int * joints, const double * refs) override;
    return_t relativeMove(int j, double delta) override;
    return_t relativeMove(const double * deltas) override;
    return_t relativeMove(int n_joint, const int * joints, const double * deltas) override;
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return_t checkMotionDone(int j, bool & flag) override;
    return_t checkMotionDone(bool & flag) override;
    return_t checkMotionDone(const std::vector<int> & joints, bool & flag) override;
    return_t setTrajSpeed(int j, double sp) override;
    return_t setTrajSpeeds(const double * spds) override;
    return_t setTrajSpeeds(int n_joint, const int * joints, const double * spds) override;
    return_t getTrajSpeed(int j, double * ref) override;
    return_t getTrajSpeeds(double * spds) override;
    return_t getTrajSpeeds(int n_joint, const int * joints, double * spds) override;
    return_t setTrajAcceleration(int j, double acc) override;
    return_t setTrajAccelerations(const double * accs) override;
    return_t setTrajAccelerations(int n_joint, const int * joints, const double * accs) override;
    return_t getTrajAcceleration(int j, double * acc) override;
    return_t getTrajAccelerations(double * accs) override;
    return_t getTrajAccelerations(int n_joint, const int * joints, double * accs) override;
#else
    return_t checkMotionDone(int j, bool * flag) override;
    return_t checkMotionDone(bool * flag) override;
    return_t checkMotionDone(int n_joint, const int * joints, bool * flags) override;
    return_t setRefSpeed(int j, double sp) override;
    return_t setRefSpeeds(const double * spds) override;
    return_t setRefSpeeds(int n_joint, const int * joints, const double * spds) override;
    return_t getRefSpeed(int j, double * ref) override;
    return_t getRefSpeeds(double * spds) override;
    return_t getRefSpeeds(int n_joint, const int * joints, double * spds) override;
    return_t setRefAcceleration(int j, double acc) override;
    return_t setRefAccelerations(const double * accs) override;
    return_t setRefAccelerations(int n_joint, const int * joints, const double * accs) override;
    return_t getRefAcceleration(int j, double * acc) override;
    return_t getRefAccelerations(double * accs) override;
    return_t getRefAccelerations(int n_joint, const int * joints, double * accs) override;
#endif
    return_t stop(int j) override;
    return_t stop() override;
    return_t stop(int n_joint, const int *joints) override;
    return_t getTargetPosition(int joint, double * ref) override;
    return_t getTargetPositions(double * refs) override;
    return_t getTargetPositions(int n_joint, const int * joints, double * refs) override;

    // ------- IPositionDirect declarations. Implementation in IPositionDirectImpl.cpp -------

    //return_t getAxes(std::size_t & ax) override;
    return_t setPosition(int j, double ref) override;
    return_t setPositions(const double * refs) override;
    return_t setPositions(int n_joint, const int * joints, const double * refs) override;
    return_t getRefPosition(int joint, double * ref) override;
    return_t getRefPositions(double * refs) override;
    return_t getRefPositions(int n_joint, const int * joints, double * refs) override;

    // --------- IPWMControl declarations. Implementation in IPWMControlImpl.cpp ---------

    //return_t getNumberOfMotors(int * number) override;
    return_t setRefDutyCycle(int m, double ref) override;
    return_t setRefDutyCycles(const double * refs) override;
    return_t getRefDutyCycle(int m, double * ref) override;
    return_t getRefDutyCycles(double * refs) override;
    return_t getDutyCycle(int m, double * val) override;
    return_t getDutyCycles(double * vals) override;

    // ----------- IRemoteVariables declarations. Implementation in IRemoteVariablesImpl.cpp --------------

    return_t getRemoteVariable(std::string key, yarp::os::Bottle & val) override;
    return_t setRemoteVariable(std::string key, const yarp::os::Bottle & val) override;
    return_t getRemoteVariablesList(yarp::os::Bottle * listOfKeys) override;

    // -------- ITorqueControl declarations. Implementation in ITorqueControlImpl.cpp --------

    //return_t getAxes(std::size_t & ax) override;
    return_t getRefTorque(int j, double * t) override;
    return_t getRefTorques(double * t) override;
    return_t setRefTorque(int j, double t) override;
    return_t setRefTorques(const double * t) override;
    return_t setRefTorques(int n_joint, const int * joints, const double * t) override;
    return_t getMotorTorqueParams(int j, yarp::dev::MotorTorqueParameters * params) override;
    return_t setMotorTorqueParams(int j, const yarp::dev::MotorTorqueParameters params) override;
    return_t getTorque(int j, double * t) override;
    return_t getTorques(double * t) override;
    return_t getTorqueRange(int j, double * min, double * max) override;
    return_t getTorqueRanges(double * min, double * max) override;

    // --------- IVelocityControl declarations. Implementation in IVelocityControlImpl.cpp ---------

    //return_t getAxes(std::size_t & ax) override;
    return_t velocityMove(int j, double spd) override;
    return_t velocityMove(const double * spds) override;
    return_t velocityMove(int n_joint, const int * joints, const double * spds) override;
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return_t getTargetVelocity(int joint, double * vel) override;
    return_t getTargetVelocities(double * vels) override;
    return_t getTargetVelocities(int n_joint, const int * joints, double * vels) override;
#else
    return_t getRefVelocity(int joint, double * vel) override;
    return_t getRefVelocities(double * vels) override;
    return_t getRefVelocities(int n_joint, const int * joints, double * vels) override;
#endif

    // ---------- IVelocityDirect declarations. Implementation in IVelocityDirectImpl.cpp ---------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return_t setRefVelocity(int jnt, double vel) override;
    return_t setRefVelocity(const std::vector<double> & vels) override;
    return_t setRefVelocity(const std::vector<int> & jnts, const std::vector<double> & vels) override;
    return_t getRefVelocity(int jnt, double & vel) override;
    return_t getRefVelocity(std::vector<double> & vels) override;
    return_t getRefVelocity(const std::vector<int> & jnts, std::vector<double> & vels) override;
#endif

    // ---------- MULTIPLE ANALOG SENSORS INTERFACES ----------

    // --------- IContactLoadCellArrays declarations. Implementation in IContactLoadCellArraysImpl.cpp ---------

    std::size_t getNrOfContactLoadCellArrays() const override;
    yarp::dev::MAS_status getContactLoadCellArrayStatus(std::size_t sens_index) const override;
    bool getContactLoadCellArrayName(std::size_t sens_index, std::string & name) const override;
    bool getContactLoadCellArrayMeasure(std::size_t sens_index, yarp::sig::Vector & out, double & timestamp) const override;
    std::size_t getContactLoadCellArraySize(std::size_t sens_index) const override;

    // --------- IEncoderArrays declarations. Implementation in IEncoderArraysImpl.cpp ---------

    std::size_t getNrOfEncoderArrays() const override;
    yarp::dev::MAS_status getEncoderArrayStatus(std::size_t sens_index) const override;
    bool getEncoderArrayName(std::size_t sens_index, std::string & name) const override;
    bool getEncoderArrayMeasure(std::size_t sens_index, yarp::sig::Vector & out, double & timestamp) const override;
    std::size_t getEncoderArraySize(std::size_t sens_index) const override;

    // --------- IOrientationSensors declarations. Implementation in IOrientationSensorsImpl.cpp ---------

    std::size_t getNrOfOrientationSensors() const override;
    yarp::dev::MAS_status getOrientationSensorStatus(std::size_t sens_index) const override;
    bool getOrientationSensorName(std::size_t sens_index, std::string & name) const override;
    bool getOrientationSensorFrameName(std::size_t sens_index, std::string & frameName) const override;
    bool getOrientationSensorMeasureAsRollPitchYaw(std::size_t sens_index, yarp::sig::Vector & rpy, double & timestamp) const override;

    // --------- IPositionSensors declarations. Implementation in IPositionSensorsImpl.cpp ---------

    std::size_t getNrOfPositionSensors() const override;
    yarp::dev::MAS_status getPositionSensorStatus(std::size_t sens_index) const override;
    bool getPositionSensorName(std::size_t sens_index, std::string & name) const override;
    bool getPositionSensorFrameName(std::size_t sens_index, std::string & frameName) const override;
    bool getPositionSensorMeasure(std::size_t sens_index, yarp::sig::Vector & xyz, double & timestamp) const override;

    // --------- ISixAxisForceTorqueSensors declarations. Implementation in ISixAxisForceTorqueSensorsImpl.cpp ---------

    std::size_t getNrOfSixAxisForceTorqueSensors() const override;
    yarp::dev::MAS_status getSixAxisForceTorqueSensorStatus(std::size_t sens_index) const override;
    bool getSixAxisForceTorqueSensorName(std::size_t sens_index, std::string & name) const override;
    bool getSixAxisForceTorqueSensorFrameName(std::size_t sens_index, std::string & frameName) const override;
    bool getSixAxisForceTorqueSensorMeasure(std::size_t sens_index, yarp::sig::Vector & out, double & timestamp) const override;

    // --------- ISkinPatches declarations. Implementation in ISkinPatchesImpl.cpp ---------

    std::size_t getNrOfSkinPatches() const override;
    yarp::dev::MAS_status getSkinPatchStatus(std::size_t sens_index) const override;
    bool getSkinPatchName(std::size_t sens_index, std::string & name) const override;
    bool getSkinPatchMeasure(std::size_t sens_index, yarp::sig::Vector & out, double & timestamp) const override;
    std::size_t getSkinPatchSize(std::size_t sens_index) const override;

    // --------- ITemperatureSensors declarations. Implementation in ITemperatureSensorsImpl.cpp ---------

    std::size_t getNrOfTemperatureSensors() const override;
    yarp::dev::MAS_status getTemperatureSensorStatus(std::size_t sens_index) const override;
    bool getTemperatureSensorName(std::size_t sens_index, std::string & name) const override;
    bool getTemperatureSensorFrameName(std::size_t sens_index, std::string & frameName) const override;
    bool getTemperatureSensorMeasure(std::size_t sens_index, double & out, double & timestamp) const override;
    bool getTemperatureSensorMeasure(std::size_t sens_index, yarp::sig::Vector & out, double & timestamp) const override;

    // --------- IThreeAxisGyroscopes declarations. Implementation in IThreeAxisGyroscopesImpl.cpp ---------

    std::size_t getNrOfThreeAxisGyroscopes() const override;
    yarp::dev::MAS_status getThreeAxisGyroscopeStatus(std::size_t sens_index) const override;
    bool getThreeAxisGyroscopeName(std::size_t sens_index, std::string & name) const override;
    bool getThreeAxisGyroscopeFrameName(std::size_t sens_index, std::string & frameName) const override;
    bool getThreeAxisGyroscopeMeasure(std::size_t sens_index, yarp::sig::Vector & out, double & timestamp) const override;

    // --------- IThreeAxisLinearAccelerometers declarations. Implementation in IThreeAxisLinearAccelerometersImpl.cpp ---------

    std::size_t getNrOfThreeAxisLinearAccelerometers() const override;
    yarp::dev::MAS_status getThreeAxisLinearAccelerometerStatus(std::size_t sens_index) const override;
    bool getThreeAxisLinearAccelerometerName(std::size_t sens_index, std::string & name) const override;
    bool getThreeAxisLinearAccelerometerFrameName(std::size_t sens_index, std::string & frameName) const override;
    bool getThreeAxisLinearAccelerometerMeasure(std::size_t sens_index, yarp::sig::Vector & out, double & timestamp) const override;

    // --------- IThreeAxisMagnetometers declarations. Implementation in IThreeAxisMagnetometersImpl.cpp ---------

    std::size_t getNrOfThreeAxisMagnetometers() const override;
    yarp::dev::MAS_status getThreeAxisMagnetometerStatus(std::size_t sens_index) const override;
    bool getThreeAxisMagnetometerName(std::size_t sens_index, std::string & name) const override;
    bool getThreeAxisMagnetometerFrameName(std::size_t sens_index, std::string & frameName) const override;
    bool getThreeAxisMagnetometerMeasure(std::size_t sens_index, yarp::sig::Vector & out, double & timestamp) const override;

private:
    roboticslab::DeviceMapper deviceMapper;

    std::vector<yarp::dev::PolyDriver *> busDevices;
    std::vector<yarp::dev::PolyDriver *> nodeDevices;
    std::vector<roboticslab::SingleBusBroker *> brokers;

    roboticslab::SyncPeriodicThread * syncThread {nullptr};
};

#endif // __CAN_BUS_BROKER_HPP__
