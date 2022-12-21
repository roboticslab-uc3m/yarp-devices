// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __TECHNOSOFT_IPOS_BASE_HPP__
#define __TECHNOSOFT_IPOS_BASE_HPP__

#include <cstdint>

#include <atomic>
#include <bitset>
#include <memory>
#include <string>

#include <yarp/conf/numeric.h>

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

#define CHECK_JOINT(j) do { int ax; if (IEncodersRaw::getAxes(&ax), (j) != ax - 1) return false; } while (0)

#define CHECK_MODE(mode) do { if ((mode) != actualControlMode) return false; } while (0)

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

    TechnosoftIposBase()
        : can(nullptr),
          iEncodersTimedRawExternal(nullptr),
          iExternalEncoderCanBusSharer(nullptr),
          monitorThread(nullptr)
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
    bool registerSender(CanSenderDelegate * sender) override;
    bool synchronize() override;

    //  --------- IAxisInfoRaw declarations. Implementation in IAxisInfoRawImpl.cpp ---------

    bool getAxisNameRaw(int axis, std::string & name) override;
    bool getJointTypeRaw(int axis, yarp::dev::JointTypeEnum & type) override;

    //  --------- IControlLimitsRaw declarations. Implementation in IControlLimitsRawImpl.cpp ---------

    bool setLimitsRaw(int axis, double min, double max) override;
    bool getLimitsRaw(int axis, double * min, double * max) override;
    bool setVelLimitsRaw(int axis, double min, double max) override;
    bool getVelLimitsRaw(int axis, double * min, double * max) override;

    //  --------- ICurrentControlRaw declarations. Implementation in ICurrentControlRawImpl.cpp ---------

    //bool getNumberOfMotorsRaw(int * number) override;
    bool getCurrentRaw(int m, double * curr) override;
    bool getCurrentsRaw(double * currs) override;
    bool getCurrentRangeRaw(int m, double * min, double * max) override;
    bool getCurrentRangesRaw(double * min, double * max) override;
    bool setRefCurrentRaw(int m, double curr) override;
    bool setRefCurrentsRaw(const double * currs) override;
    bool setRefCurrentsRaw(int n_motor, const int * motors, const double * currs) override;
    bool getRefCurrentRaw(int m, double * curr) override;
    bool getRefCurrentsRaw(double * currs) override;

    //  ---------- IEncodersRaw declarations. Implementation in IEncodersRawImpl.cpp ----------

    bool getAxes(int * ax) override;
    bool resetEncoderRaw(int j) override;
    bool resetEncodersRaw() override;
    bool setEncoderRaw(int j, double val) override;
    bool setEncodersRaw(const double * vals) override;
    bool getEncoderRaw(int j, double * v) override;
    bool getEncodersRaw(double * encs) override;
    bool getEncoderSpeedRaw(int j, double * sp) override;
    bool getEncoderSpeedsRaw(double * spds) override;
    bool getEncoderAccelerationRaw(int j, double * spds) override;
    bool getEncoderAccelerationsRaw(double * accs) override;

    //  ---------- IEncodersTimedRaw declarations. Implementation in IEncodersRawImpl.cpp ----------

    bool getEncoderTimedRaw(int j, double * encs, double * time) override;
    bool getEncodersTimedRaw(double * encs, double * time) override;

    //  ---------- IImpedanceControlRaw declarations. Implementation in IImpedanceControlRawImpl.cpp ----------

    //bool getAxes(int * ax) override;

    bool getImpedanceRaw(int j, double * stiffness, double * damping) override
    { return false; }

    bool setImpedanceRaw(int j, double stiffness, double damping) override
    { return false; }

    bool setImpedanceOffsetRaw(int j, double offset) override
    { return false; }

    bool getImpedanceOffsetRaw(int j, double * offset) override
    { return false; }

    bool getCurrentImpedanceLimitRaw(int j, double * min_stiff, double * max_stiff, double * min_damp, double * max_damp) override
    { return false; }

    //  ---------- IInteractionModeRaw declarations. Implementation in IInteractionModeRawImpl.cpp ----------

    bool getInteractionModeRaw(int axis, yarp::dev::InteractionModeEnum * mode) override
    { return false; }

    bool getInteractionModesRaw(int n_joints, int * joints, yarp::dev::InteractionModeEnum * modes) override
    { return false; }

    bool getInteractionModesRaw(yarp::dev::InteractionModeEnum * modes) override
    { return false; }

    bool setInteractionModeRaw(int axis, yarp::dev::InteractionModeEnum mode) override
    { return false; }

    bool setInteractionModesRaw(int n_joints, int * joints, yarp::dev::InteractionModeEnum * modes) override
    { return false; }

    bool setInteractionModesRaw(yarp::dev::InteractionModeEnum * modes) override
    { return false; }

    //  ---------- IJointFaultRaw declarations. Implementation in IJointFaultRawImpl.cpp ----------

    bool getLastJointFaultRaw(int j, int & fault, std::string & message) override;

    //  --------- IMotorRaw declarations. Implementation in IMotorRawImpl.cpp ---------

    bool getNumberOfMotorsRaw(int * num) override;
    bool getTemperatureRaw(int m, double * val) override;
    bool getTemperaturesRaw(double * vals) override;
    bool getTemperatureLimitRaw(int m, double * temp) override;
    bool setTemperatureLimitRaw(int m, double temp) override;
    bool getGearboxRatioRaw(int m, double * val) override;
    bool setGearboxRatioRaw(int m, double val) override;

    //  --------- IMotorEncodersRaw declarations. Implementation in IMotorEncodersRawImpl.cpp ---------

    bool getNumberOfMotorEncodersRaw(int * num) override;
    bool resetMotorEncoderRaw(int m) override;
    bool resetMotorEncodersRaw() override;
    bool setMotorEncoderCountsPerRevolutionRaw(int m, double cpr) override;
    bool getMotorEncoderCountsPerRevolutionRaw(int m, double * cpr) override;
    bool setMotorEncoderRaw(int m, double val) override;
    bool setMotorEncodersRaw(const double * vals) override;
    bool getMotorEncoderRaw(int m, double * v) override;
    bool getMotorEncodersRaw(double * encs) override;
    bool getMotorEncoderTimedRaw(int m, double * encs, double * stamp) override;
    bool getMotorEncodersTimedRaw(double * encs, double * stamps) override;
    bool getMotorEncoderSpeedRaw(int m, double * sp) override;
    bool getMotorEncoderSpeedsRaw(double * spds) override;
    bool getMotorEncoderAccelerationRaw(int m, double * spds) override;
    bool getMotorEncoderAccelerationsRaw(double * vaccs) override;

    //  --------- IPidControlRaw declarations. Implementation in IPidControlRawImpl.cpp ---------

    bool setPidRaw(const yarp::dev::PidControlTypeEnum & pidtype, int j, const yarp::dev::Pid & pid) override
    { return false; }

    bool setPidsRaw(const yarp::dev::PidControlTypeEnum & pidtype, const yarp::dev::Pid * pids) override
    { return false; }

    bool setPidReferenceRaw(const yarp::dev::PidControlTypeEnum & pidtype, int j, double ref) override
    { return false; }

    bool setPidReferencesRaw(const yarp::dev::PidControlTypeEnum & pidtype, const double * refs) override
    { return false; }

    bool setPidErrorLimitRaw(const yarp::dev::PidControlTypeEnum & pidtype, int j, double limit) override
    { return false; }

    bool setPidErrorLimitsRaw(const yarp::dev::PidControlTypeEnum & pidtype, const double * limits) override
    { return false; }

    bool getPidErrorRaw(const yarp::dev::PidControlTypeEnum & pidtype, int j, double * err) override
    { return false; }

    bool getPidErrorsRaw(const yarp::dev::PidControlTypeEnum & pidtype, double * errs) override
    { return false; }

    bool getPidOutputRaw(const yarp::dev::PidControlTypeEnum & pidtype, int j, double * out) override
    { return false; }

    bool getPidOutputsRaw(const yarp::dev::PidControlTypeEnum & pidtype, double * outs) override
    { return false; }

    bool getPidRaw(const yarp::dev::PidControlTypeEnum & pidtype, int j, yarp::dev::Pid * pid) override
    { return false; }

    bool getPidsRaw(const yarp::dev::PidControlTypeEnum & pidtype, yarp::dev::Pid * pids) override
    { return false; }

    bool getPidReferenceRaw(const yarp::dev::PidControlTypeEnum & pidtype, int j, double * ref) override
    { return false; }

    bool getPidReferencesRaw(const yarp::dev::PidControlTypeEnum & pidtype, double * refs) override
    { return false; }

    bool getPidErrorLimitRaw(const yarp::dev::PidControlTypeEnum & pidtype, int j, double * limit) override
    { return false; }

    bool getPidErrorLimitsRaw(const yarp::dev::PidControlTypeEnum & pidtype, double * limits) override
    { return false; }

    bool resetPidRaw(const yarp::dev::PidControlTypeEnum & pidtype, int j) override
    { return false; }

    bool disablePidRaw(const yarp::dev::PidControlTypeEnum & pidtype, int j) override
    { return false; }

    bool enablePidRaw(const yarp::dev::PidControlTypeEnum & pidtype, int j) override
    { return false; }

    bool setPidOffsetRaw(const yarp::dev::PidControlTypeEnum & pidtype, int j, double v) override
    { return false; }

    bool isPidEnabledRaw(const yarp::dev::PidControlTypeEnum & pidtype, int j, bool * enabled) override
    { return false; }

    // ------- IRemoteVariablesRaw declarations. Implementation in IRemoteVariablesRawImpl.cpp -------

    bool getRemoteVariableRaw(std::string key, yarp::os::Bottle & val) override
    { return false; }

    bool setRemoteVariableRaw(std::string key, const yarp::os::Bottle & val) override
    { return false; }

    bool getRemoteVariablesListRaw(yarp::os::Bottle * listOfKeys) override
    { return false; }

    // -------- ITorqueControlRaw declarations. Implementation in ITorqueControlRawImpl.cpp --------

    //bool getAxes(int * ax) override;
    bool getRefTorqueRaw(int j, double * t) override;
    bool getRefTorquesRaw(double * t) override;
    bool setRefTorqueRaw(int j, double t) override;
    bool setRefTorquesRaw(const double * t) override;
    bool getTorqueRaw(int j, double * t) override;
    bool getTorquesRaw(double * t) override;
    bool getTorqueRangeRaw(int j, double * min, double * max) override;
    bool getTorqueRangesRaw(double * min, double * max) override;
    bool getMotorTorqueParamsRaw(int j, yarp::dev::MotorTorqueParameters * params) override;
    bool setMotorTorqueParamsRaw(int j, const yarp::dev::MotorTorqueParameters params) override;

protected:

    enum report_level { NONE, INFO, WARN, FAULT };

    struct report_storage
    {
        const char * reg;
        const std::bitset<16> & actual;
        const std::bitset<16> & stored;
    };

    bool reportBitToggle(report_storage report, int level, std::size_t pos, const char * msgSet, const char * msgReset = nullptr);

    virtual void interpretModesOfOperation(std::int8_t modesOfOperation);
    virtual void interpretIpStatus(std::uint16_t ipStatus) {}
    virtual void reset();

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

    CanOpenNode * can;
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

    // read only after initial configuration, conceptually immutable

    yarp::conf::vocab32_t initialMode {0};

    double drivePeakCurrent {0.0};
    double samplingPeriod {0.0};

    std::string axisName;
    yarp::conf::vocab32_t jointType {0};

    bool reverse {false};

    PdoConfiguration tpdo1Conf;
    PdoConfiguration tpdo2Conf;
    PdoConfiguration tpdo3Conf;

    double heartbeatPeriod {0.0};
    double syncPeriod {0.0};

    unsigned int canId {0};

private:

    //! Make sure stored variables actually make sense.
    bool validateInitialState();

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

    bool setLimitRaw(double limit, bool isMin);
    bool getLimitRaw(double * limit, bool isMin);

    yarp::dev::PolyDriver externalEncoderDevice;
    yarp::dev::IEncodersTimedRaw * iEncodersTimedRawExternal;
    roboticslab::ICanBusSharer * iExternalEncoderCanBusSharer;

    yarp::os::Timer * monitorThread;
};

} // namespace roboticslab

#endif // __TECHNOSOFT_IPOS_BASE_HPP__
