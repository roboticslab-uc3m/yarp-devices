// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __TECHNOSOFT_IPOS_HPP__
#define __TECHNOSOFT_IPOS_HPP__

#include <yarp/os/Timer.h>

#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/IAxisInfo.h>
#include <yarp/dev/IControlLimits.h>
#include <yarp/dev/IControlMode.h>
#include <yarp/dev/ICurrentControl.h>
#include <yarp/dev/IEncodersTimed.h>
#include <yarp/dev/IJointFault.h>
#include <yarp/dev/IMotor.h>
#include <yarp/dev/IMotorEncoders.h>
#include <yarp/dev/IPositionControl.h>
#include <yarp/dev/IPositionDirect.h>
#include <yarp/dev/IRemoteVariables.h>
#include <yarp/dev/ITorqueControl.h>
#include <yarp/dev/IVelocityControl.h>
#include <yarp/dev/PolyDriver.h>

#include "CanOpenNode.hpp"
#include "CommandBuffer.hpp"
#include "ICanBusSharer.hpp"

#include "InterpolatedPositionBuffer.hpp"
#include "StateVariables.hpp"

#define CHECK_JOINT(j) do { int ax; if (getAxes(&ax), (j) != ax - 1) return false; } while (0)

#define CHECK_MODE(mode) do { if ((mode) != vars.actualControlMode) return false; } while (0)

namespace roboticslab
{

/**
 * @ingroup YarpPlugins
 * @defgroup TechnosoftIpos
 * @brief Contains roboticslab::TechnosoftIpos.
 */

/**
 * @ingroup TechnosoftIpos
 * @brief Custom EMCY messages.
 */
class TechnosoftIposEmcy : public EmcyCodeRegistry
{
public:
    std::string codeToMessage(std::uint16_t code) override;
};

/**
 * @ingroup TechnosoftIpos
 * @brief Implementation for the Technosoft iPOS as a single CAN bus joint (controlboard raw interfaces).
 */
class TechnosoftIpos : public yarp::dev::DeviceDriver,
                       public yarp::dev::IAxisInfoRaw,
                       public yarp::dev::IControlLimitsRaw,
                       public yarp::dev::IControlModeRaw,
                       public yarp::dev::ICurrentControlRaw,
                       public yarp::dev::IEncodersTimedRaw,
                       public yarp::dev::IJointFaultRaw,
                       public yarp::dev::IMotorRaw,
                       public yarp::dev::IMotorEncodersRaw,
                       public yarp::dev::IPositionControlRaw,
                       public yarp::dev::IPositionDirectRaw,
                       public yarp::dev::IRemoteVariablesRaw,
                       public yarp::dev::ITorqueControlRaw,
                       public yarp::dev::IVelocityControlRaw,
                       public ICanBusSharer
{
public:

    TechnosoftIpos()
        : can(nullptr),
          iEncodersTimedRawExternal(nullptr),
          iExternalEncoderCanBusSharer(nullptr),
          ipBuffer(nullptr),
          monitorThread(nullptr)
    { }

    ~TechnosoftIpos() override
    { close(); }

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
    bool setLimitRaw(double limit, bool isMin);
    bool getLimitRaw(double * limit, bool isMin);

    //  --------- IControlModeRaw declarations. Implementation in IControlModeRawImpl.cpp ---------

    bool getControlModeRaw(int j, int * mode) override;
    bool getControlModesRaw(int * modes) override;
    bool getControlModesRaw(int n_joint, const int * joints, int * modes) override;
    bool setControlModeRaw(int j, int mode) override;
    bool setControlModesRaw(int * modes) override;
    bool setControlModesRaw(int n_joint, const int * joints, int * modes) override;

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

    class report_storage;
    bool reportBitToggle(report_storage report, int level, std::size_t pos, const char * msgSet, const char * msgReset = nullptr);

    void interpretMsr(std::uint16_t msr);
    void interpretMer(std::uint16_t mer);
    void interpretDer(std::uint16_t der);
    void interpretDer2(std::uint16_t der2);
    void interpretCer(std::uint16_t cer);
    void interpretStatusword(std::uint16_t statusword);
    void interpretModesOfOperation(std::int8_t modesOfOperation);
    void interpretIpStatus(std::uint16_t ipStatus);

    void handleTpdo1(std::uint16_t statusword, std::uint16_t msr, std::int8_t modesOfOperation);
    void handleTpdo2(std::uint16_t mer, std::uint16_t der);
    void handleTpdo3(std::int32_t position, std::int16_t current);
    void handleEmcy(EmcyConsumer::code_t code, std::uint8_t reg, const std::uint8_t * msef);
    void handleNmt(NmtState state);

    bool monitorWorker(const yarp::os::YarpTimerEvent & event);

    CanOpenNode * can;

    yarp::dev::PolyDriver externalEncoderDevice;
    yarp::dev::IEncodersTimedRaw * iEncodersTimedRawExternal;
    roboticslab::ICanBusSharer * iExternalEncoderCanBusSharer;

    StateVariables vars;

    CommandBuffer commandBuffer;

    InterpolatedPositionBuffer * ipBuffer;

    yarp::os::Timer * monitorThread;
};

} // namespace roboticslab

#endif // __TECHNOSOFT_IPOS_HPP__
