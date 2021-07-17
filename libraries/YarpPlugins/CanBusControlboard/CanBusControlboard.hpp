// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __CAN_BUS_CONTROLBOARD_HPP__
#define __CAN_BUS_CONTROLBOARD_HPP__

#include <vector>

#include <yarp/dev/ControlBoardInterfaces.h>

#include "DeviceMapper.hpp"
#include "CanBusBroker.hpp"
#include "SyncPeriodicThread.hpp"

#define CHECK_JOINT(j) do { int n = deviceMapper.getControlledAxes(); if ((j) < 0 || (j) > n - 1) return false; } while (0)

namespace roboticslab
{

/**
 * @ingroup YarpPlugins
 * @defgroup CanBusControlboard
 * @brief Contains roboticslab::CanBusControlboard.
 */

/**
 * @ingroup CanBusControlboard
 * @brief CAN-oriented control board that implements all YARP motor interfaces.
 *
 * This controlboardwrapper2 subdevice exposes motor commands to CAN nodes
 * modelled as wrapped motor raw subdevices (i.e. devices which implement the
 * xxxRaw interface counterparts). At the core of this driver class, another set
 * of wrapped devices implements hardware connection to a physical CAN bus, thus
 * allowing CAN reads and writes that CanBusControlboard manages asynchronously
 * with regard to exposed YARP commands (see @ref CanReaderWriterThread).
 *
 * This device also supports fake CAN buses and fake CAN nodes, see
 * <a href="https://github.com/roboticslab-uc3m/yarp-devices/issues/241#issuecomment-569112698">instructions</a>.
 */
class CanBusControlboard : public yarp::dev::DeviceDriver,
                           public yarp::dev::IAmplifierControl,
                           public yarp::dev::IAxisInfo,
                           public yarp::dev::IControlCalibration,
                           public yarp::dev::IControlLimits,
                           public yarp::dev::IControlMode,
                           public yarp::dev::ICurrentControl,
                           public yarp::dev::IEncodersTimed,
                           public yarp::dev::IImpedanceControl,
                           public yarp::dev::IInteractionMode,
                           public yarp::dev::IMotor,
                           public yarp::dev::IMotorEncoders,
                           public yarp::dev::IPidControl,
                           public yarp::dev::IPositionControl,
                           public yarp::dev::IPositionDirect,
                           public yarp::dev::IPWMControl,
                           public yarp::dev::IRemoteVariables,
                           public yarp::dev::ITorqueControl,
                           public yarp::dev::IVelocityControl
{
public:
    ~CanBusControlboard() override
    { close(); }

    // -------- DeviceDriver declarations. Implementation in DeviceDriverImpl.cpp --------

    bool open(yarp::os::Searchable & config) override;
    bool close() override;

    //  --------- IAmplifierControl declarations. Implementation in IAmplifierControlImpl.cpp ---------

    bool enableAmp(int j) override;
    bool disableAmp(int j) override;
    bool getAmpStatus(int j, int * v) override;
    bool getAmpStatus(int * st) override;
    //bool getCurrent(int j, double * val) override;
    //bool getCurrents(double * vals) override;
    bool getMaxCurrent(int j, double * v) override;
    bool setMaxCurrent(int j, double v) override;
    bool getNominalCurrent(int m, double * val) override;
    bool setNominalCurrent(int m, double val) override;
    bool getPeakCurrent(int m, double * val) override;
    bool setPeakCurrent(int m, double val) override;
    bool getPWM(int j, double * val) override;
    bool getPWMLimit(int j, double * val) override;
    bool setPWMLimit(int j, double val) override;
    bool getPowerSupplyVoltage(int j, double * val) override;

    //  --------- IAxisInfo declarations. Implementation in IAxisInfoImpl.cpp ---------

    bool getAxisName(int axis, std::string & name) override;
    bool getJointType(int axis, yarp::dev::JointTypeEnum & type) override;

    //  --------- IControlCalibration declarations. Implementation in IControlCalibrationImpl.cpp ---------

    bool calibrateAxisWithParams(int axis, unsigned int type, double p1, double p2, double p3) override;
    bool setCalibrationParameters(int axis, const yarp::dev::CalibrationParameters & params) override;
    bool calibrationDone(int j) override;

    //  --------- IControlLimits declarations. Implementation in IControlLimitsImpl.cpp ---------

    bool setLimits(int axis, double min, double max) override;
    bool getLimits(int axis, double * min, double * max) override;
    bool setVelLimits(int axis, double min, double max) override;
    bool getVelLimits(int axis, double * min, double * max) override;

    //  --------- IControlMode declarations. Implementation in IControlModeImpl.cpp ---------

    bool getControlMode(int j, int * mode) override;
    bool getControlModes(int * modes) override;
    bool getControlModes(int n_joint, const int * joints, int * modes) override;
    bool setControlMode(int j, const int mode) override;
    bool setControlModes(int n_joint, const int * joints, int * modes) override;
    bool setControlModes(int * modes) override;

    //  --------- ICurrentControl declarations. Implementation in ICurrentControlImpl.cpp ---------

    //bool getNumberOfMotors(int * ax) override;
    bool getCurrent(int m, double * curr) override;
    bool getCurrents(double * currs) override;
    bool getCurrentRange(int m, double * min, double * max) override;
    bool getCurrentRanges(double * mins, double * maxs) override;
    bool setRefCurrent(int m, double curr) override;
    bool setRefCurrents(const double * currs) override;
    bool setRefCurrents(int n_motor, const int * motors, const double * currs) override;
    bool getRefCurrent(int m, double * curr) override;
    bool getRefCurrents(double * currs) override;

    //  ---------- IEncoders declarations. Implementation in IEncodersImpl.cpp ----------

    bool getAxes(int * ax) override;
    bool resetEncoder(int j) override;
    bool resetEncoders() override;
    bool setEncoder(int j, double val) override;
    bool setEncoders(const double * vals) override;
    bool getEncoder(int j, double * v) override;
    bool getEncoders(double * encs) override;
    bool getEncoderSpeed(int j, double * spd) override;
    bool getEncoderSpeeds(double * spds) override;
    bool getEncoderAcceleration(int j, double * spds) override;
    bool getEncoderAccelerations(double * accs) override;

    //  ---------- IEncodersTimed declarations. Implementation in IEncodersImpl.cpp ----------

    bool getEncoderTimed(int j, double * encs, double * time) override;
    bool getEncodersTimed(double * encs, double * times) override;

    //  --------- IImpedanceControl declarations. Implementation in IImpedanceControlImpl.cpp ---------

    //bool getAxes(int * ax) override;
    bool getImpedance(int j, double * stiffness, double * damping) override;
    bool setImpedance(int j, double stiffness, double damping) override;
    bool setImpedanceOffset(int j, double offset) override;
    bool getImpedanceOffset(int j, double * offset) override;
    bool getCurrentImpedanceLimit(int j, double * min_stiff, double * max_stiff, double * min_damp, double * max_damp) override;

    // -----------IInteractionMode declarations. Implementation in IInteractionModeImpl.cpp --------------

    bool getInteractionMode(int axis, yarp::dev::InteractionModeEnum * mode) override;
    bool getInteractionModes(yarp::dev::InteractionModeEnum * modes) override;
    bool getInteractionModes(int n_joints, int * joints, yarp::dev::InteractionModeEnum * modes) override;
    bool setInteractionMode(int axis, yarp::dev::InteractionModeEnum mode) override;
    bool setInteractionModes(yarp::dev::InteractionModeEnum * modes) override;
    bool setInteractionModes(int n_joints, int * joints, yarp::dev::InteractionModeEnum * modes) override;

    //  --------- IMotor declarations. Implementation in IMotorImpl.cpp ---------

    bool getNumberOfMotors(int * num) override;
    bool getTemperature(int m, double * val) override;
    bool getTemperatures(double * vals) override;
    bool getTemperatureLimit(int m, double * temp) override;
    bool setTemperatureLimit(int m, double temp) override;
    bool getGearboxRatio(int m, double * val) override;
    bool setGearboxRatio(int m, double val) override;

    //  --------- IMotorEncoders declarations. Implementation in IMotorEncodersImpl.cpp ---------

    bool getNumberOfMotorEncoders(int * num) override;
    bool resetMotorEncoder(int m) override;
    bool resetMotorEncoders() override;
    bool setMotorEncoderCountsPerRevolution(int m, double cpr) override;
    bool getMotorEncoderCountsPerRevolution(int m, double * cpr) override;
    bool setMotorEncoder(int m, double val) override;
    bool setMotorEncoders(const double * vals) override;
    bool getMotorEncoder(int m, double * v) override;
    bool getMotorEncoders(double * encs) override;
    bool getMotorEncoderTimed(int m, double * enc, double * stamp) override;
    bool getMotorEncodersTimed(double * encs, double * stamps) override;
    bool getMotorEncoderSpeed(int m, double * sp) override;
    bool getMotorEncoderSpeeds(double *spds) override;
    bool getMotorEncoderAcceleration(int m, double * acc) override;
    bool getMotorEncoderAccelerations(double * accs) override;

    //  --------- IPidControl declarations. Implementation in IPidControlImpl.cpp ---------

    bool setPid(const yarp::dev::PidControlTypeEnum & pidtype, int j, const yarp::dev::Pid & pid) override;
    bool setPids(const yarp::dev::PidControlTypeEnum & pidtype, const yarp::dev::Pid * pids) override;
    bool setPidReference(const yarp::dev::PidControlTypeEnum & pidtype, int j, double ref) override;
    bool setPidReferences(const yarp::dev::PidControlTypeEnum & pidtype, const double * refs) override;
    bool setPidErrorLimit(const yarp::dev::PidControlTypeEnum & pidtype, int j, double limit) override;
    bool setPidErrorLimits(const yarp::dev::PidControlTypeEnum & pidtype, const double * limits) override;
    bool getPidError(const yarp::dev::PidControlTypeEnum & pidtype, int j, double * err) override;
    bool getPidErrors(const yarp::dev::PidControlTypeEnum & pidtype, double * errs) override;
    bool getPidOutput(const yarp::dev::PidControlTypeEnum & pidtype, int j, double * out) override;
    bool getPidOutputs(const yarp::dev::PidControlTypeEnum & pidtype, double * outs) override;
    bool getPid(const yarp::dev::PidControlTypeEnum & pidtype, int j, yarp::dev::Pid * pid) override;
    bool getPids(const yarp::dev::PidControlTypeEnum & pidtype, yarp::dev::Pid * pids) override;
    bool getPidReference(const yarp::dev::PidControlTypeEnum & pidtype, int j, double * ref) override;
    bool getPidReferences(const yarp::dev::PidControlTypeEnum & pidtype, double * refs) override;
    bool getPidErrorLimit(const yarp::dev::PidControlTypeEnum & pidtype, int j, double * limit) override;
    bool getPidErrorLimits(const yarp::dev::PidControlTypeEnum & pidtype, double * limits) override;
    bool resetPid(const yarp::dev::PidControlTypeEnum & pidtype, int j) override;
    bool disablePid(const yarp::dev::PidControlTypeEnum & pidtype, int j) override;
    bool enablePid(const yarp::dev::PidControlTypeEnum & pidtype, int j) override;
    bool setPidOffset(const yarp::dev::PidControlTypeEnum & pidtype, int j, double v) override;
    bool isPidEnabled(const yarp::dev::PidControlTypeEnum & pidtype, int j, bool * enabled) override;

    // ------- IPositionControl declarations. Implementation in IPositionControlImpl.cpp -------

    //bool getAxes(int * ax) override;
    bool positionMove(int j, double ref) override;
    bool positionMove(const double * refs) override;
    bool positionMove(int n_joint, const int * joints, const double * refs) override;
    bool relativeMove(int j, double delta) override;
    bool relativeMove(const double * deltas) override;
    bool relativeMove(int n_joint, const int * joints, const double * deltas) override;
    bool checkMotionDone(int j, bool * flag) override;
    bool checkMotionDone(bool * flag) override;
    bool checkMotionDone(int n_joint, const int * joints, bool * flag) override;
    bool setRefSpeed(int j, double sp) override;
    bool setRefSpeeds(const double * spds) override;
    bool setRefSpeeds(int n_joint, const int * joints, const double * spds) override;
    bool setRefAcceleration(int j, double acc) override;
    bool setRefAccelerations(const double * accs) override;
    bool setRefAccelerations(int n_joint, const int * joints, const double * accs) override;
    bool getRefSpeed(int j, double * spd) override;
    bool getRefSpeeds(double * spds) override;
    bool getRefSpeeds(int n_joint, const int * joints, double * spds) override;
    bool getRefAcceleration(int j, double * acc) override;
    bool getRefAccelerations(double * accs) override;
    bool getRefAccelerations(int n_joint, const int * joints, double * accs) override;
    bool stop(int j) override;
    bool stop() override;
    bool stop(int n_joint, const int *joints) override;
    bool getTargetPosition(int joint, double * ref) override;
    bool getTargetPositions(double * refs) override;
    bool getTargetPositions(int n_joint, const int * joints, double * refs) override;

    // ------- IPositionDirect declarations. Implementation in IPositionDirectImpl.cpp -------

    //bool getAxes(int * ax) override;
    bool setPosition(int j, double ref) override;
    bool setPositions(const double * refs) override;
    bool setPositions(int n_joint, const int * joints, const double * refs) override;
    bool getRefPosition(int joint, double * ref) override;
    bool getRefPositions(double * refs) override;
    bool getRefPositions(int n_joint, const int * joints, double * refs) override;

    //  --------- IPWMControl declarations. Implementation in IPWMControlImpl.cpp ---------

    //bool getNumberOfMotors(int * number) override;
    bool setRefDutyCycle(int m, double ref) override;
    bool setRefDutyCycles(const double * refs) override;
    bool getRefDutyCycle(int m, double * ref) override;
    bool getRefDutyCycles(double * refs) override;
    bool getDutyCycle(int m, double * val) override;
    bool getDutyCycles(double * vals) override;

    // -----------IRemoteVariables declarations. Implementation in IRemoteVariablesImpl.cpp --------------

    bool getRemoteVariable(std::string key, yarp::os::Bottle & val) override;
    bool setRemoteVariable(std::string key, const yarp::os::Bottle & val) override;
    bool getRemoteVariablesList(yarp::os::Bottle * listOfKeys) override;

    // -------- ITorqueControl declarations. Implementation in ITorqueControlImpl.cpp --------

    //bool getAxes(int * ax) override;
    bool getRefTorque(int j, double * t) override;
    bool getRefTorques(double * t) override;
    bool setRefTorque(int j, double t) override;
    bool setRefTorques(const double * t) override;
    bool setRefTorques(int n_joint, const int * joints, const double * t) override;
    bool getMotorTorqueParams(int j, yarp::dev::MotorTorqueParameters * params) override;
    bool setMotorTorqueParams(int j, const yarp::dev::MotorTorqueParameters params) override;
    bool getTorque(int j, double * t) override;
    bool getTorques(double * t) override;
    bool getTorqueRange(int j, double * min, double * max) override;
    bool getTorqueRanges(double * min, double * max) override;

    //  --------- IVelocityControl declarations. Implementation in IVelocityControlImpl.cpp ---------

    //bool getAxes(int * ax) override;
    bool velocityMove(int j, double spd) override;
    bool velocityMove(const double * spds) override;
    bool velocityMove(int n_joint, const int * joints, const double * spds) override;
    bool getRefVelocity(int joint, double * vel) override;
    bool getRefVelocities(double * vels) override;
    bool getRefVelocities(int n_joint, const int * joints, double * vels) override;
    //bool setRefAcceleration(int j, double acc) override;
    //bool setRefAccelerations(const double * accs) override;
    //bool setRefAccelerations(int n_joint, const int * joints, const double * accs) override;
    //bool getRefAcceleration(int j, double * acc) override;
    //bool getRefAccelerations(double * accs) override;
    //bool getRefAccelerations(int n_joint, const int * joints, double * accs) override;
    //bool stop(int j) override;
    //bool stop() override;
    //bool stop(int n_joint, const int *joints) override;

private:
    DeviceMapper deviceMapper;

    std::vector<yarp::dev::PolyDriver *> busDevices;
    std::vector<yarp::dev::PolyDriver *> nodeDevices;
    std::vector<CanBusBroker *> canBusBrokers;

    SyncPeriodicThread * syncThread {nullptr};
};

} // namespace roboticslab

#endif // __CAN_BUS_CONTROLBOARD_HPP__
