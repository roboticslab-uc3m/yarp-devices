// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __CAN_BUS_CONTROLBOARD_HPP__
#define __CAN_BUS_CONTROLBOARD_HPP__

#include <vector>

#include <yarp/dev/CanBusInterface.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/PolyDriverList.h>

#include <ColorDebug.h>

#include "ICanBusSharer.hpp"
#include "PositionDirectThread.hpp"
#include "DeviceMapper.hpp"
#include "CanRxTxThreads.hpp"

#define CHECK_JOINT(j) do { int n = deviceMapper.getControlledAxes(); if ((j) < 0 || (j) > n - 1) return false; } while (0)

#define DEFAULT_MODE "position"
#define DEFAULT_CUI_TIMEOUT 1.0
#define DEFAULT_CAN_BUS "CanBusHico"
#define DEFAULT_LIN_INTERP_PERIOD_MS 50
#define DEFAULT_LIN_INTERP_BUFFER_SIZE 1
#define DEFAULT_LIN_INTERP_MODE "pt"

#define DEFAULT_CAN_RX_BUFFER_SIZE 500
#define DEFAULT_CAN_TX_BUFFER_SIZE 500
#define DEFAULT_CAN_RX_PERIOD_MS -1
#define DEFAULT_CAN_TX_PERIOD_MS 1.0
#define DEFAULT_CAN_SDO_TIMEOUT_MS 25.0
#define DEFAULT_CAN_DRIVE_STATE_TIMEOUT 2.5

namespace roboticslab
{

/**
 * @ingroup YarpPlugins
 * \defgroup CanBusControlboard
 * @brief Contains roboticslab::CanBusControlboard.
 */

/**
 * @ingroup CanBusControlboard
 * @brief Implements all motor interfaces.
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

    CanBusControlboard()
        : iCanBus(nullptr), canReaderThread(nullptr), canWriterThread(nullptr),
          posdThread(nullptr), linInterpPeriodMs(0), linInterpBufferSize(0)
    { }

    // -------- DeviceDriver declarations. Implementation in DeviceDriverImpl.cpp --------

    virtual bool open(yarp::os::Searchable & config) override;
    virtual bool close() override;

    //  --------- IAmplifierControl declarations. Implementation in IAmplifierControlImpl.cpp ---------

    virtual bool enableAmp(int j) override;
    virtual bool disableAmp(int j) override;
    virtual bool getAmpStatus(int j, int * v) override;
    virtual bool getAmpStatus(int * st) override;
    //virtual bool getCurrent(int j, double * val) override;
    //virtual bool getCurrents(double * vals) override;
    virtual bool getMaxCurrent(int j, double * v) override;
    virtual bool setMaxCurrent(int j, double v) override;
    virtual bool getNominalCurrent(int m, double * val) override;
    virtual bool setNominalCurrent(int m, double val) override;
    virtual bool getPeakCurrent(int m, double * val) override;
    virtual bool setPeakCurrent(int m, double val) override;
    virtual bool getPWM(int j, double * val) override;
    virtual bool getPWMLimit(int j, double * val) override;
    virtual bool setPWMLimit(int j, double val) override;
    virtual bool getPowerSupplyVoltage(int j, double * val) override;

    //  --------- IAxisInfo declarations. Implementation in IAxisInfoImpl.cpp ---------

    virtual bool getAxisName(int axis, std::string & name) override;
    virtual bool getJointType(int axis, yarp::dev::JointTypeEnum & type) override;

    //  --------- IControlCalibration declarations. Implementation in IControlCalibrationImpl.cpp ---------

    virtual bool calibrateAxisWithParams(int axis, unsigned int type, double p1, double p2, double p3) override;
    virtual bool setCalibrationParameters(int axis, const yarp::dev::CalibrationParameters & params) override;
    virtual bool calibrationDone(int j) override;

    //  --------- IControlLimits declarations. Implementation in IControlLimitsImpl.cpp ---------

    virtual bool setLimits(int axis, double min, double max) override;
    virtual bool getLimits(int axis, double * min, double * max) override;
    virtual bool setVelLimits(int axis, double min, double max) override;
    virtual bool getVelLimits(int axis, double * min, double * max) override;

    //  --------- IControlMode declarations. Implementation in IControlModeImpl.cpp ---------

    virtual bool getControlMode(int j, int * mode) override;
    virtual bool getControlModes(int * modes) override;
    virtual bool getControlModes(int n_joint, const int * joints, int * modes) override;
    virtual bool setControlMode(int j, const int mode) override;
    virtual bool setControlModes(int n_joint, const int * joints, int * modes) override;
    virtual bool setControlModes(int * modes) override;

    //  --------- ICurrentControl declarations. Implementation in ICurrentControlImpl.cpp ---------

    //virtual bool getNumberOfMotors(int * ax) override;
    virtual bool getCurrent(int m, double * curr) override;
    virtual bool getCurrents(double * currs) override;
    virtual bool getCurrentRange(int m, double * min, double * max) override;
    virtual bool getCurrentRanges(double * min, double * max) override;
    virtual bool setRefCurrent(int m, double curr) override;
    virtual bool setRefCurrents(const double * currs) override;
    virtual bool setRefCurrents(int n_motor, const int * motors, const double * currs) override;
    virtual bool getRefCurrents(double * currs) override;
    virtual bool getRefCurrent(int m, double *curr) override;

    //  ---------- IEncoders declarations. Implementation in IEncodersImpl.cpp ----------

    virtual bool getAxes(int * ax) override;
    virtual bool resetEncoder(int j) override;
    virtual bool resetEncoders() override;
    virtual bool setEncoder(int j, double val) override;
    virtual bool setEncoders(const double * vals) override;
    virtual bool getEncoder(int j, double * v) override;
    virtual bool getEncoders(double * encs) override;
    virtual bool getEncoderSpeed(int j, double * spd) override;
    virtual bool getEncoderSpeeds(double * spds) override;
    virtual bool getEncoderAcceleration(int j, double * spds) override;
    virtual bool getEncoderAccelerations(double * accs) override;

    //  ---------- IEncodersTimed declarations. Implementation in IEncodersImpl.cpp ----------

    virtual bool getEncoderTimed(int j, double * encs, double * time) override;
    virtual bool getEncodersTimed(double * encs, double * times) override;

    //  --------- IImpedanceControl declarations. Implementation in IImpedanceControlImpl.cpp ---------

    //virtual bool getAxes(int * ax) override;
    virtual bool getImpedance(int j, double * stiffness, double * damping) override;
    virtual bool setImpedance(int j, double stiffness, double damping) override;
    virtual bool setImpedanceOffset(int j, double offset) override;
    virtual bool getImpedanceOffset(int j, double * offset) override;
    virtual bool getCurrentImpedanceLimit(int j, double * min_stiff, double * max_stiff, double * min_damp, double * max_damp) override;

    // -----------IInteractionMode declarations. Implementation in IInteractionModeImpl.cpp --------------

    virtual bool getInteractionMode(int axis, yarp::dev::InteractionModeEnum * mode) override;
    virtual bool getInteractionModes(yarp::dev::InteractionModeEnum * modes) override;
    virtual bool getInteractionModes(int n_joints, int * joints, yarp::dev::InteractionModeEnum * modes) override;
    virtual bool setInteractionMode(int axis, yarp::dev::InteractionModeEnum mode) override;
    virtual bool setInteractionModes(yarp::dev::InteractionModeEnum * modes) override;
    virtual bool setInteractionModes(int n_joints, int * joints, yarp::dev::InteractionModeEnum * modes) override;

    //  --------- IMotor declarations. Implementation in IMotorImpl.cpp ---------

    virtual bool getNumberOfMotors(int *num) override;
    virtual bool getTemperature(int m, double * val) override;
    virtual bool getTemperatures(double * vals) override;
    virtual bool getTemperatureLimit(int m, double * temp) override;
    virtual bool setTemperatureLimit(int m, double temp) override;
    virtual bool getGearboxRatio(int m, double * val) override;
    virtual bool setGearboxRatio(int m, double val) override;

    //  --------- IMotorEncoders declarations. Implementation in IMotorEncodersImpl.cpp ---------

    virtual bool getNumberOfMotorEncoders(int * num) override;
    virtual bool resetMotorEncoder(int m) override;
    virtual bool resetMotorEncoders() override;
    virtual bool setMotorEncoderCountsPerRevolution(int m, double cpr) override;
    virtual bool getMotorEncoderCountsPerRevolution(int m, double * cpr) override;
    virtual bool setMotorEncoder(int m, double val) override;
    virtual bool setMotorEncoders(const double * vals) override;
    virtual bool getMotorEncoder(int m, double * v) override;
    virtual bool getMotorEncoders(double * encs) override;
    virtual bool getMotorEncoderTimed(int m, double * enc, double * stamp) override;
    virtual bool getMotorEncodersTimed(double * encs, double * stamps) override;
    virtual bool getMotorEncoderSpeed(int m, double * sp) override;
    virtual bool getMotorEncoderSpeeds(double *spds) override;
    virtual bool getMotorEncoderAcceleration(int m, double * acc) override;
    virtual bool getMotorEncoderAccelerations(double * accs) override;

    //  --------- IPidControl declarations. Implementation in IPidControlImpl.cpp ---------

    virtual bool setPid(const yarp::dev::PidControlTypeEnum & pidtype, int j, const yarp::dev::Pid & pid) override;
    virtual bool setPids(const yarp::dev::PidControlTypeEnum & pidtype, const yarp::dev::Pid * pids) override;
    virtual bool setPidReference(const yarp::dev::PidControlTypeEnum & pidtype, int j, double ref) override;
    virtual bool setPidReferences(const yarp::dev::PidControlTypeEnum & pidtype, const double * refs) override;
    virtual bool setPidErrorLimit(const yarp::dev::PidControlTypeEnum & pidtype, int j, double limit) override;
    virtual bool setPidErrorLimits(const yarp::dev::PidControlTypeEnum & pidtype, const double * limits) override;
    virtual bool getPidError(const yarp::dev::PidControlTypeEnum & pidtype, int j, double * err) override;
    virtual bool getPidErrors(const yarp::dev::PidControlTypeEnum & pidtype, double * errs) override;
    virtual bool getPidOutput(const yarp::dev::PidControlTypeEnum & pidtype, int j, double *out) override;
    virtual bool getPidOutputs(const yarp::dev::PidControlTypeEnum & pidtype, double * outs) override;
    virtual bool getPid(const yarp::dev::PidControlTypeEnum & pidtype, int j, yarp::dev::Pid * pid) override;
    virtual bool getPids(const yarp::dev::PidControlTypeEnum & pidtype, yarp::dev::Pid * pids) override;
    virtual bool getPidReference(const yarp::dev::PidControlTypeEnum & pidtype, int j, double * ref) override;
    virtual bool getPidReferences(const yarp::dev::PidControlTypeEnum & pidtype, double * refs) override;
    virtual bool getPidErrorLimit(const yarp::dev::PidControlTypeEnum & pidtype, int j, double * limit) override;
    virtual bool getPidErrorLimits(const yarp::dev::PidControlTypeEnum & pidtype, double * limits) override;
    virtual bool resetPid(const yarp::dev::PidControlTypeEnum & pidtype, int j) override;
    virtual bool disablePid(const yarp::dev::PidControlTypeEnum & pidtype, int j) override;
    virtual bool enablePid(const yarp::dev::PidControlTypeEnum & pidtype, int j) override;
    virtual bool setPidOffset(const yarp::dev::PidControlTypeEnum & pidtype, int j, double v) override;
    virtual bool isPidEnabled(const yarp::dev::PidControlTypeEnum & pidtype, int j, bool * enabled) override;

    // ------- IPositionControl declarations. Implementation in IPositionControlImpl.cpp -------

    //virtual bool getAxes(int * ax) override;
    virtual bool positionMove(int j, double ref) override;
    virtual bool positionMove(const double * refs) override;
    virtual bool positionMove(int n_joint, const int * joints, const double * refs) override;
    virtual bool relativeMove(int j, double delta) override;
    virtual bool relativeMove(const double * deltas) override;
    virtual bool relativeMove(int n_joint, const int * joints, const double * deltas) override;
    virtual bool checkMotionDone(int j, bool * flag) override;
    virtual bool checkMotionDone(bool * flag) override;
    virtual bool checkMotionDone(int n_joint, const int * joints, bool * flag) override;
    virtual bool setRefSpeed(int j, double sp) override;
    virtual bool setRefSpeeds(const double * spds) override;
    virtual bool setRefSpeeds(int n_joint, const int * joints, const double * spds) override;
    virtual bool setRefAcceleration(int j, double acc) override;
    virtual bool setRefAccelerations(const double * accs) override;
    virtual bool setRefAccelerations(int n_joint, const int * joints, const double * accs) override;
    virtual bool getRefSpeeds(int n_joint, const int * joints, double * spds) override;
    virtual bool getRefSpeed(int j, double * ref) override;
    virtual bool getRefSpeeds(double * spds) override;
    virtual bool getRefAcceleration(int j, double * acc) override;
    virtual bool getRefAccelerations(double * accs) override;
    virtual bool getRefAccelerations(int n_joint, const int * joints, double * accs) override;
    virtual bool stop(int j) override;
    virtual bool stop() override;
    virtual bool stop(int n_joint, const int *joints) override;
    virtual bool getTargetPosition(int joint, double * ref) override;
    virtual bool getTargetPositions(double * refs) override;
    virtual bool getTargetPositions(int n_joint, const int * joints, double * refs) override;

    // ------- IPositionDirect declarations. Implementation in IPositionDirectImpl.cpp -------

    //virtual bool getAxes(int * ax) override;
    virtual bool setPosition(int j, double ref) override;
    virtual bool setPositions(const double * refs) override;
    virtual bool setPositions(int n_joint, const int * joints, const double * refs) override;
    virtual bool getRefPosition(int joint, double * ref) override;
    virtual bool getRefPositions(double *refs) override;
    virtual bool getRefPositions(int n_joint, const int * joints, double * refs) override;

    //  --------- IPWMControl declarations. Implementation in IPWMControlImpl.cpp ---------

    //virtual bool getNumberOfMotors(int * number) override;
    virtual bool setRefDutyCycle(int m, double ref) override;
    virtual bool setRefDutyCycles(const double * refs) override;
    virtual bool getRefDutyCycle(int m, double * ref) override;
    virtual bool getRefDutyCycles(double * refs) override;
    virtual bool getDutyCycle(int m, double * val) override;
    virtual bool getDutyCycles(double * vals) override;

    // -----------IRemoteVariables declarations. Implementation in IRemoteVariablesImpl.cpp --------------

    virtual bool getRemoteVariable(std::string key, yarp::os::Bottle & val) override;
    virtual bool setRemoteVariable(std::string key, const yarp::os::Bottle & val) override;
    virtual bool getRemoteVariablesList(yarp::os::Bottle * listOfKeys) override;

    // -------- ITorqueControl declarations. Implementation in ITorqueControlImpl.cpp --------

    //virtual bool getAxes(int * ax) override;
    virtual bool getRefTorques(double *t);
    virtual bool getRefTorque(int j, double *t);
    virtual bool setRefTorques(const double *t);
    virtual bool setRefTorque(int j, double t);
    virtual bool setRefTorques(const int n_joint, const int *joints, const double *t);
    virtual bool getMotorTorqueParams(int j,  yarp::dev::MotorTorqueParameters *params);
    virtual bool setMotorTorqueParams(int j, const yarp::dev::MotorTorqueParameters params);
    virtual bool getTorque(int j, double *t);
    virtual bool getTorques(double *t);
    virtual bool getTorqueRange(int j, double *min, double *max);
    virtual bool getTorqueRanges(double *min, double *max);

    //  --------- IVelocityControl declarations. Implementation in IVelocityControlImpl.cpp ---------

    //virtual bool getAxes(int * ax) override;
    virtual bool velocityMove(int j, double spd) override;
    virtual bool velocityMove(const double * spds) override;
    virtual bool velocityMove(int n_joint, const int * joints, const double * spds) override;
    virtual bool getRefVelocity(int joint, double * vel) override;
    virtual bool getRefVelocities(double * vels) override;
    virtual bool getRefVelocities(int n_joint, const int * joints, double * vels) override;
    //virtual bool setRefAcceleration(int j, double acc) override;
    //virtual bool setRefAccelerations(const double * accs) override;
    //virtual bool setRefAccelerations(int n_joint, const int * joints, const double * accs) override;
    //virtual bool getRefAcceleration(int j, double * acc) override;
    //virtual bool getRefAccelerations(double * accs) override;
    //virtual bool getRefAccelerations(int n_joint, const int * joints, double * accs) override;
    //virtual bool stop(int j) override;
    //virtual bool stop() override;
    //virtual bool stop(int n_joint, const int *joints) override;

private:

    yarp::dev::PolyDriver canBusDevice;
    yarp::dev::ICanBus * iCanBus;

    yarp::dev::PolyDriverList nodeDevices;

    DeviceMapper deviceMapper;
    std::vector<ICanBusSharer *> iCanBusSharers;

    CanReaderThread * canReaderThread;
    CanWriterThread * canWriterThread;

    PositionDirectThread * posdThread;
    int linInterpPeriodMs;
    int linInterpBufferSize;
    std::string linInterpMode;
};

} // namespace roboticslab

#endif //  __CAN_BUS_CONTROLBOARD_HPP__
