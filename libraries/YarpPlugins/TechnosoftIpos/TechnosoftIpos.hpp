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
#include <yarp/dev/IMotor.h>
#include <yarp/dev/IMotorEncoders.h>
#include <yarp/dev/IPositionControl.h>
#include <yarp/dev/IPositionDirect.h>
#include <yarp/dev/IRemoteVariables.h>
#include <yarp/dev/ITorqueControl.h>
#include <yarp/dev/IVelocityControl.h>
#include <yarp/dev/PolyDriver.h>

#include "CanOpen.hpp"
#include "ICanBusSharer.hpp"
#include "LinearInterpolationBuffer.hpp"
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
    virtual std::string codeToMessage(std::uint16_t code) override;
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
          linInterpBuffer(nullptr),
          monitorThread(nullptr)
    { }

    ~TechnosoftIpos()
    { close(); }

    //  --------- DeviceDriver declarations. Implementation in DeviceDriverImpl.cpp ---------

    virtual bool open(yarp::os::Searchable & config) override;
    virtual bool close() override;

    //  --------- ICanBusSharer declarations. Implementation in ICanBusSharerImpl.cpp ---------

    virtual unsigned int getId() override;
    virtual std::vector<unsigned int> getAdditionalIds() override;
    virtual bool interpretMessage(const yarp::dev::CanMessage & message) override;
    virtual bool initialize() override;
    virtual bool finalize() override;
    virtual bool registerSender(CanSenderDelegate * sender) override;

    //  --------- IAxisInfoRaw declarations. Implementation in IAxisInfoRawImpl.cpp ---------

    virtual bool getAxisNameRaw(int axis, std::string & name) override;
    virtual bool getJointTypeRaw(int axis, yarp::dev::JointTypeEnum & type) override;

    //  --------- IControlLimitsRaw declarations. Implementation in IControlLimitsRawImpl.cpp ---------

    virtual bool setLimitsRaw(int axis, double min, double max) override;
    virtual bool getLimitsRaw(int axis, double * min, double * max) override;
    virtual bool setVelLimitsRaw(int axis, double min, double max) override;
    virtual bool getVelLimitsRaw(int axis, double * min, double * max) override;
    bool setLimitRaw(double limit, bool isMin);
    bool getLimitRaw(double * limit, bool isMin);

    //  --------- IControlModeRaw declarations. Implementation in IControlModeRawImpl.cpp ---------

    virtual bool getControlModeRaw(int j, int * mode) override;
    virtual bool getControlModesRaw(int * modes) override;
    virtual bool getControlModesRaw(int n_joint, const int * joints, int * modes) override;
    virtual bool setControlModeRaw(int j, int mode) override;
    bool setPositionDirectModeRaw();
    virtual bool setControlModesRaw(int * modes) override;
    virtual bool setControlModesRaw(int n_joint, const int * joints, int * modes) override;

    //  --------- ICurrentControlRaw declarations. Implementation in ICurrentControlRawImpl.cpp ---------

    //virtual bool getNumberOfMotorsRaw(int * number) override;
    virtual bool getCurrentRaw(int m, double * curr) override;
    virtual bool getCurrentsRaw(double * currs) override;
    virtual bool getCurrentRangeRaw(int m, double * min, double * max) override;
    virtual bool getCurrentRangesRaw(double * min, double * max) override;
    virtual bool setRefCurrentRaw(int m, double curr) override;
    virtual bool setRefCurrentsRaw(const double * currs) override;
    virtual bool setRefCurrentsRaw(int n_motor, const int * motors, const double * currs) override;
    virtual bool getRefCurrentRaw(int m, double * curr) override;
    virtual bool getRefCurrentsRaw(double * currs) override;

    //  ---------- IEncodersRaw declarations. Implementation in IEncodersRawImpl.cpp ----------

    virtual bool getAxes(int * ax) override;
    virtual bool resetEncoderRaw(int j) override;
    virtual bool resetEncodersRaw() override;
    virtual bool setEncoderRaw(int j, double val) override;
    virtual bool setEncodersRaw(const double * vals) override;
    virtual bool getEncoderRaw(int j, double * v) override;
    virtual bool getEncodersRaw(double * encs) override;
    virtual bool getEncoderSpeedRaw(int j, double * sp) override;
    virtual bool getEncoderSpeedsRaw(double * spds) override;
    virtual bool getEncoderAccelerationRaw(int j, double * spds) override;
    virtual bool getEncoderAccelerationsRaw(double * accs) override;

    //  ---------- IEncodersTimedRaw declarations. Implementation in IEncodersRawImpl.cpp ----------

    virtual bool getEncoderTimedRaw(int j, double * encs, double * time) override;
    virtual bool getEncodersTimedRaw(double * encs, double * time) override;

    //  --------- IMotorRaw declarations. Implementation in IMotorRawImpl.cpp ---------

    virtual bool getNumberOfMotorsRaw(int * num) override;
    virtual bool getTemperatureRaw(int m, double * val) override;
    virtual bool getTemperaturesRaw(double * vals) override;
    virtual bool getTemperatureLimitRaw(int m, double * temp) override;
    virtual bool setTemperatureLimitRaw(int m, double temp) override;
    virtual bool getGearboxRatioRaw(int m, double * val) override;
    virtual bool setGearboxRatioRaw(int m, double val) override;

    //  --------- IMotorEncodersRaw declarations. Implementation in IMotorEncodersRawImpl.cpp ---------

    virtual bool getNumberOfMotorEncodersRaw(int * num) override;
    virtual bool resetMotorEncoderRaw(int m) override;
    virtual bool resetMotorEncodersRaw() override;
    virtual bool setMotorEncoderCountsPerRevolutionRaw(int m, double cpr) override;
    virtual bool getMotorEncoderCountsPerRevolutionRaw(int m, double * cpr) override;
    virtual bool setMotorEncoderRaw(int m, double val) override;
    virtual bool setMotorEncodersRaw(const double * vals) override;
    virtual bool getMotorEncoderRaw(int m, double * v) override;
    virtual bool getMotorEncodersRaw(double * encs) override;
    virtual bool getMotorEncoderTimedRaw(int m, double * encs, double * stamp) override;
    virtual bool getMotorEncodersTimedRaw(double * encs, double * stamps) override;
    virtual bool getMotorEncoderSpeedRaw(int m, double * sp) override;
    virtual bool getMotorEncoderSpeedsRaw(double * spds) override;
    virtual bool getMotorEncoderAccelerationRaw(int m, double * spds) override;
    virtual bool getMotorEncoderAccelerationsRaw(double * vaccs) override;

    // ------- IPositionControlRaw declarations. Implementation in IPositionControlRawImpl.cpp -------

    //virtual bool getAxes(int * ax) override;
    virtual bool positionMoveRaw(int j, double ref) override;
    virtual bool positionMoveRaw(const double * refs) override;
    virtual bool positionMoveRaw(int n_joint, const int * joints, const double * refs) override;
    virtual bool relativeMoveRaw(int j, double delta) override;
    virtual bool relativeMoveRaw(const double * deltas) override;
    virtual bool relativeMoveRaw(int n_joint, const int * joints, const double * deltas) override;
    virtual bool checkMotionDoneRaw(int j, bool * flag) override;
    virtual bool checkMotionDoneRaw(bool * flag) override;
    virtual bool checkMotionDoneRaw(int n_joint, const int * joints, bool * flag) override;
    virtual bool setRefSpeedRaw(int j, double sp) override;
    virtual bool setRefSpeedsRaw(const double * spds) override;
    virtual bool setRefSpeedsRaw(int n_joint, const int * joints, const double * spds) override;
    virtual bool setRefAccelerationRaw(int j, double acc) override;
    virtual bool setRefAccelerationsRaw(const double * accs) override;
    virtual bool setRefAccelerationsRaw(int n_joint, const int * joints, const double * accs) override;
    virtual bool getRefSpeedRaw(int j, double * ref) override;
    virtual bool getRefSpeedsRaw(double * spds) override;
    virtual bool getRefSpeedsRaw(int n_joint, const int * joints, double * spds) override;
    virtual bool getRefAccelerationRaw(int j, double * acc) override;
    virtual bool getRefAccelerationsRaw(double * accs) override;
    virtual bool getRefAccelerationsRaw(int n_joint, const int * joints, double * accs) override;
    virtual bool stopRaw(int j) override;
    virtual bool stopRaw() override;
    virtual bool stopRaw(int n_joint, const int * joints) override;
    virtual bool getTargetPositionRaw(int joint, double * ref) override;
    virtual bool getTargetPositionsRaw(double * refs) override;
    virtual bool getTargetPositionsRaw(int n_joint, const int * joints, double * refs) override;

    // ------- IPositionDirectRaw declarations. Implementation in IPositionDirectRawImpl.cpp -------

    //virtual bool getAxes(int * ax) override;
    virtual bool setPositionRaw(int j, double ref) override;
    virtual bool setPositionsRaw(const double * refs) override;
    virtual bool setPositionsRaw(int n_joint, const int * joints, const double * refs) override;
    virtual bool getRefPositionRaw(int joint, double * ref) override;
    virtual bool getRefPositionsRaw(double * refs) override;
    virtual bool getRefPositionsRaw(int n_joint, const int *joints, double * refs) override;

    // ------- IRemoteVariablesRaw declarations. Implementation in IRemoteVariablesRawImpl.cpp -------

    virtual bool getRemoteVariableRaw(std::string key, yarp::os::Bottle & val) override;
    virtual bool setRemoteVariableRaw(std::string key, const yarp::os::Bottle & val) override;
    virtual bool getRemoteVariablesListRaw(yarp::os::Bottle * listOfKeys) override;

    // -------- ITorqueControlRaw declarations. Implementation in ITorqueControlRawImpl.cpp --------

    //virtual bool getAxes(int * ax) override;
    virtual bool getRefTorqueRaw(int j, double * t) override;
    virtual bool getRefTorquesRaw(double * t) override;
    virtual bool setRefTorqueRaw(int j, double t) override;
    virtual bool setRefTorquesRaw(const double * t) override;
    virtual bool getTorqueRaw(int j, double * t) override;
    virtual bool getTorquesRaw(double * t) override;
    virtual bool getTorqueRangeRaw(int j, double * min, double * max) override;
    virtual bool getTorqueRangesRaw(double * min, double * max) override;
    virtual bool getMotorTorqueParamsRaw(int j, yarp::dev::MotorTorqueParameters * params) override;
    virtual bool setMotorTorqueParamsRaw(int j, const yarp::dev::MotorTorqueParameters params) override;

    //  --------- IVelocityControlRaw declarations. Implementation in IVelocityControlRawImpl.cpp ---------

    //virtual bool getAxes(int * ax) override;
    virtual bool velocityMoveRaw(int j, double sp) override;
    virtual bool velocityMoveRaw(const double * sp) override;
    virtual bool velocityMoveRaw(int n_joint, const int * joints, const double * spds) override;
    virtual bool getRefVelocityRaw(int joint, double * vel) override;
    virtual bool getRefVelocitiesRaw(double * vels) override;
    virtual bool getRefVelocitiesRaw(int n_joint, const int * joints, double * vels) override;
    //virtual bool setRefAccelerationRaw(int j, double acc) override;
    //virtual bool setRefAccelerationsRaw(const double * accs) override;
    //virtual bool setRefAccelerationsRaw(int n_joint, const int * joints, const double * accs) override;
    //virtual bool getRefAccelerationRaw(int j, double * acc) override;
    //virtual bool getRefAccelerationsRaw(double * accs) override;
    //virtual bool getRefAccelerationsRaw(int n_joint, const int * joints, double * accs) override;
    //virtual bool stopRaw(int j) override;
    //virtual bool stopRaw() override;
    //virtual bool stopRaw(int n_joint, const int *joints) override;

private:

    void interpretSupportedDriveModes(std::uint32_t data);
    void interpretMsr(std::uint16_t msr);
    void interpretMer(std::uint16_t mer);
    void interpretDer(std::uint16_t der);
    void interpretDer2(std::uint16_t der2);
    void interpretCer(std::uint16_t cer);
    void interpretStatusword(std::uint16_t statusword);
    void interpretModesOfOperation(std::int8_t modesOfOperation);
    void interpretPtStatus(std::uint16_t ptStatus);

    void handleTpdo1(std::uint16_t statusword, std::uint16_t msr, std::int8_t modesOfOperation);
    void handleTpdo2(std::uint16_t mer, std::uint16_t der);
    void handleTpdo3(std::int32_t position, std::int16_t current);
    void handleEmcy(EmcyConsumer::code_t code, std::uint8_t reg, const std::uint8_t * msef);
    void handleNmt(NmtState state);

    bool monitorWorker(const yarp::os::YarpTimerEvent & event);

    bool quitHaltState(int mode);

    CanOpen * can;

    yarp::dev::PolyDriver externalEncoderDevice;
    yarp::dev::IEncodersTimedRaw * iEncodersTimedRawExternal;
    roboticslab::ICanBusSharer * iExternalEncoderCanBusSharer;

    StateVariables vars;

    LinearInterpolationBuffer * linInterpBuffer;

    yarp::os::Timer * monitorThread;
};

} // namespace roboticslab

#endif // __TECHNOSOFT_IPOS_HPP__
