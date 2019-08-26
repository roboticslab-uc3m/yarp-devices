// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __TECHNOSOFT_IPOS__
#define __TECHNOSOFT_IPOS__

#include <stdint.h>
#include <cmath>
#include <mutex>
#include <sstream>

#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <yarp/dev/IControlLimits.h>
#include <yarp/dev/IRemoteVariables.h>

#include <ColorDebug.h>

#include "ICanBusSharer.hpp"
#include "CanOpen.hpp"
#include "ITechnosoftIpos.h"
#include "LinearInterpolationBuffer.hpp"

#define CHECK_JOINT(j) do { int ax; if (getAxes(&ax), (j) != ax - 1) return false; } while (0)

namespace roboticslab
{

/**
 * @ingroup YarpPlugins
 * \defgroup TechnosoftIpos
 * @brief Contains roboticslab::TechnosoftIpos.
 */

/**
 * @ingroup TechnosoftIpos
 * @brief Stores last encoder reads, obtains mean speeds and accelerations
 * via differentiation.
 */
class EncoderRead
{
public:
    EncoderRead(double initialPos);
    void update(double newPos, double newTime = 0.0);
    double queryPosition() const;
    double querySpeed() const;
    double queryAcceleration() const;
    double queryTime() const;

private:
    double lastPosition, nextToLastPosition;
    double lastSpeed, nextToLastSpeed;
    double lastAcceleration;
    yarp::os::Stamp lastStamp;
    mutable std::mutex encoderMutex;
};

class TechnosoftIposEmcy : public EmcyCodeRegistry
{
public:
    virtual std::string codeToMessage(std::uint16_t code);
};

/**
* @ingroup TechnosoftIpos
* @brief Implementation for the Technosoft iPOS as a single CAN bus joint (controlboard raw interfaces).
*
*/
class TechnosoftIpos : public yarp::dev::DeviceDriver,
                       public yarp::dev::IControlLimitsRaw,
                       public yarp::dev::IControlModeRaw,
                       public yarp::dev::ICurrentControlRaw,
                       public yarp::dev::IEncodersTimedRaw,
                       public yarp::dev::IInteractionModeRaw,
                       public yarp::dev::IPositionControlRaw,
                       public yarp::dev::IPositionDirectRaw,
                       public yarp::dev::IRemoteVariablesRaw,
                       public yarp::dev::ITorqueControlRaw,
                       public yarp::dev::IVelocityControlRaw,
                       public ICanBusSharer,
                       public ITechnosoftIpos
{
public:

    TechnosoftIpos()
        : canId(0),
          sender(0),
          can(0),
          iEncodersTimedRawExternal(0),
          lastEncoderRead(0.0),
          modeCurrentTorque(0),
          drivePeakCurrent(0.0),
          linInterpBuffer(0),
          maxVel(0.0),
          tr(0.0),
          k(0.0),
          encoderPulses(0),
          pulsesPerSample(0)
    {}

    //  --------- DeviceDriver Declarations. Implementation in TechnosoftIpos.cpp ---------
    virtual bool open(yarp::os::Searchable& config);
    virtual bool close();

    //  --------- ICanBusSharer Declarations. Implementation in TechnosoftIpos.cpp ---------
    virtual bool setIEncodersTimedRawExternal(IEncodersTimedRaw * iEncodersTimedRaw); // -- ??
    virtual bool interpretMessage(const yarp::dev::CanMessage & message);
    virtual bool initialize();
    /** "start". Figure 5.1 Drive’s status machine. States and transitions (p68, 84/263). */
    virtual bool start();
    /** "ready to switch on", also acts as "shutdown" */
    virtual bool readyToSwitchOn();
    /** "switch on", also acts as "disable operation" */
    virtual bool switchOn();
    /** enable */
    virtual bool enable();
    /** recoverFromError */
    virtual bool recoverFromError();
    /** reset node */
    virtual bool resetNode(int id);
    /** reset all nodes */
    virtual bool resetNodes();
    /** reset communications */
    virtual bool resetCommunication();
    /** send new point to PT/PVT buffer */
    virtual bool sendLinearInterpolationTarget();
    /** send start signal to PT/PVT mode */
    virtual bool sendLinearInterpolationStart();
    virtual bool registerSender(CanSenderDelegate * sender);

    //  --------- IControlLimitsRaw Declarations. Implementation in IControlLimitsRawImpl.cpp ---------
    virtual bool setLimitsRaw(int axis, double min, double max);
    virtual bool getLimitsRaw(int axis, double *min, double *max);
    virtual bool setVelLimitsRaw(int axis, double min, double max);
    virtual bool getVelLimitsRaw(int axis, double *min, double *max);
    //-- Auxiliary functions
    bool setLimitRaw(double limit, bool isMin);
    bool getLimitRaw(double * limit, bool isMin);

    //  --------- IControlModeRaw Declarations. Implementation in IControlModeRawImpl.cpp ---------
    bool setPositionModeRaw(int j);
    bool setVelocityModeRaw(int j);
    bool setPositionDirectModeRaw();
    bool setTorqueModeRaw(int j);

    virtual bool getControlModeRaw(int j, int *mode);
    //-- Auxiliary functions (splitted) of getControlModeRaw
    bool getControlModeRaw1(int *mode);
    bool getControlModeRaw2();
    bool getControlModeRaw3();
    bool getControlModeRaw4();

    virtual bool getControlModesRaw(int *modes);

    virtual bool getControlModesRaw(const int n_joint, const int *joints, int *modes);
    virtual bool setControlModeRaw(const int j, const int mode);
    virtual bool setControlModesRaw(const int n_joint, const int *joints, int *modes);
    virtual bool setControlModesRaw(int *modes);

    //  --------- ICurrentControlRaw Declarations. Implementation in ICurrentControlRawImpl.cpp ---------
    virtual bool getNumberOfMotorsRaw(int *number);
    virtual bool getCurrentRaw(int m, double *curr);
    virtual bool getCurrentsRaw(double *currs);
    virtual bool getCurrentRangeRaw(int m, double *min, double *max);
    virtual bool getCurrentRangesRaw(double *min, double *max);
    virtual bool setRefCurrentsRaw(const double *currs);
    virtual bool setRefCurrentRaw(int m, double curr);
    virtual bool setRefCurrentsRaw(const int n_motor, const int *motors, const double *currs);
    virtual bool getRefCurrentsRaw(double *currs);
    virtual bool getRefCurrentRaw(int m, double *curr);

    //  ---------- IEncodersRaw Declarations. Implementation in IEncodersRawImpl.cpp ----------
    virtual bool resetEncoderRaw(int j);
    virtual bool resetEncodersRaw();
    virtual bool setEncoderRaw(int j, double val);
    virtual bool setEncodersRaw(const double *vals);
    virtual bool getEncoderRaw(int j, double *v);
    virtual bool getEncodersRaw(double *encs);
    virtual bool getEncoderSpeedRaw(int j, double *sp);
    virtual bool getEncoderSpeedsRaw(double *spds);
    virtual bool getEncoderAccelerationRaw(int j, double *spds);
    virtual bool getEncoderAccelerationsRaw(double *accs);

    //  ---------- IEncodersTimedRaw Declarations. Implementation in IEncodersTimedRawImpl.cpp ----------
    virtual bool getEncodersTimedRaw(double *encs, double *time);
    virtual bool getEncoderTimedRaw(int j, double *encs, double *time);

    // ------- IPositionControlRaw declarations. Implementation in IPositionControlRawImpl.cpp -------
    virtual bool getAxes(int *ax);
    virtual bool positionMoveRaw(int j, double ref);
    virtual bool positionMoveRaw(const double *refs);
    virtual bool relativeMoveRaw(int j, double delta);
    virtual bool relativeMoveRaw(const double *deltas);
    virtual bool checkMotionDoneRaw(int j, bool *flag);
    virtual bool checkMotionDoneRaw(bool *flag);
    virtual bool setRefSpeedRaw(int j, double sp);
    virtual bool setRefSpeedsRaw(const double *spds);
    virtual bool setRefAccelerationRaw(int j, double acc);
    virtual bool setRefAccelerationsRaw(const double *accs);
    virtual bool getRefSpeedRaw(int j, double *ref);
    virtual bool getRefSpeedsRaw(double *spds);
    virtual bool getRefAccelerationRaw(int j, double *acc);
    virtual bool getRefAccelerationsRaw(double *accs);
    virtual bool stopRaw(int j);
    virtual bool stopRaw();
    virtual bool positionMoveRaw(const int n_joint, const int *joints, const double *refs);
    virtual bool relativeMoveRaw(const int n_joint, const int *joints, const double *deltas);
    virtual bool checkMotionDoneRaw(const int n_joint, const int *joints, bool *flags);
    virtual bool setRefSpeedsRaw(const int n_joint, const int *joints, const double *spds);
    virtual bool setRefAccelerationsRaw(const int n_joint, const int *joints, const double *accs);
    virtual bool getRefSpeedsRaw(const int n_joint, const int *joints, double *spds);
    virtual bool getRefAccelerationsRaw(const int n_joint, const int *joints, double *accs);
    virtual bool stopRaw(const int n_joint, const int *joints);
    virtual bool getTargetPositionRaw(const int joint, double *ref);
    virtual bool getTargetPositionsRaw(double *refs);
    virtual bool getTargetPositionsRaw(const int n_joint, const int *joints, double *refs);

    // ------- IPositionDirectRaw declarations. Implementation in IPositionDirectRawImpl.cpp -------
    virtual bool setPositionRaw(int j, double ref);
    virtual bool setPositionsRaw(const int n_joint, const int *joints, const double *refs);
    virtual bool setPositionsRaw(const double *refs);
    virtual bool getRefPositionRaw(const int joint, double *ref);
    virtual bool getRefPositionsRaw(double *refs);
    virtual bool getRefPositionsRaw(const int n_joint, const int *joints, double *refs);

    // -------- ITorqueControlRaw declarations. Implementation in ITorqueControlRawImpl.cpp --------
    virtual bool getRefTorquesRaw(double *t);
    virtual bool getRefTorqueRaw(int j, double *t);
    virtual bool setRefTorquesRaw(const double *t);
    virtual bool setRefTorqueRaw(int j, double t);
    virtual bool getTorqueRaw(int j, double *t);
    virtual bool getTorquesRaw(double *t);
    virtual bool getTorqueRangeRaw(int j, double *min, double *max);
    virtual bool getTorqueRangesRaw(double *min, double *max);
    virtual bool getMotorTorqueParamsRaw(int j, yarp::dev::MotorTorqueParameters *params);
    virtual bool setMotorTorqueParamsRaw(int j, const yarp::dev::MotorTorqueParameters params);

    //  --------- IVelocityControlRaw Declarations. Implementation in IVelocityControlRawImpl.cpp ---------
    virtual bool velocityMoveRaw(int j, double sp);
    virtual bool velocityMoveRaw(const double *sp);
    virtual bool velocityMoveRaw(const int n_joint, const int *joints, const double *spds);
    virtual bool getRefVelocityRaw(const int joint, double *vel);
    virtual bool getRefVelocitiesRaw(double *vels);
    virtual bool getRefVelocitiesRaw(const int n_joint, const int *joints, double *vels);
    // ------------------- Just declareted in IPositionControl2Raw
    // -- virtual bool setRefAccelerationsRaw(const int n_joint, const int *joints, const double *accs);
    // ------------------- Just declareted in IPositionControl2Raw
    // -- virtual bool getRefAccelerationsRaw(const int n_joint, const int *joints, double *accs);
    // ------------------- Just declareted in IPositionControl2Raw
    // -- virtual bool stopRaw(const int n_joint, const int *joints);

    // ------- IInteractionModeRaw declarations. Implementation in IInteractionModeRawImpl.cpp -------
    virtual bool getInteractionModeRaw(int axis, yarp::dev::InteractionModeEnum* mode);
    virtual bool getInteractionModesRaw(int n_joints, int *joints, yarp::dev::InteractionModeEnum* modes);
    virtual bool getInteractionModesRaw(yarp::dev::InteractionModeEnum* modes);
    virtual bool setInteractionModeRaw(int axis, yarp::dev::InteractionModeEnum mode);
    virtual bool setInteractionModesRaw(int n_joints, int *joints, yarp::dev::InteractionModeEnum* modes);
    virtual bool setInteractionModesRaw(yarp::dev::InteractionModeEnum* modes);

    // ------- IRemoteVariablesRaw declarations. Implementation in IRemoteVariablesRawImpl.cpp -------
    virtual bool getRemoteVariableRaw(std::string key, yarp::os::Bottle& val);
    virtual bool setRemoteVariableRaw(std::string key, const yarp::os::Bottle& val);
    virtual bool getRemoteVariablesListRaw(yarp::os::Bottle* listOfKeys);

protected:

    // return -1 for negative numbers, +1 for positive numbers, 0 for zero
    // https://stackoverflow.com/a/4609795
    template<typename T>
    inline int sgn(T val)
    { return (T(0) < val) - (val < T(0)); }

    int32_t degreesToInternalUnits(double value, int derivativeOrder = 0)
    { return value * tr * (encoderPulses / 360.0) * std::pow(1.0 / pulsesPerSample, derivativeOrder); }

    double internalUnitsToDegrees(int32_t value, int derivativeOrder = 0)
    { return value / (tr * (encoderPulses / 360.0) * std::pow(1.0 / pulsesPerSample, derivativeOrder)); }

    int16_t currentToInternalUnits(double value)
    { return value * sgn(tr) * 65520.0 / (2.0 * drivePeakCurrent); }

    double internalUnitsToCurrent(int16_t value)
    { return value * sgn(tr) * 2.0 * drivePeakCurrent / 65520.0; }

    double internalUnitsToPeakCurrent(int16_t value)
    { return 2.0 * drivePeakCurrent * (32767.0 - value) / 65520.0; }

    double currentToTorque(double current)
    { return current * std::abs(tr) * k; }

    double torqueToCurrent(double torque)
    { return torque / (std::abs(tr) * k); }

    int canId;
    CanSenderDelegate * sender;
    CanOpen * can;

    //-- Encoder stuff
    yarp::dev::IEncodersTimedRaw* iEncodersTimedRawExternal;
    EncoderRead lastEncoderRead;

    //-- Current stuff
    int modeCurrentTorque;
    double drivePeakCurrent;

    //-- PT/PVT stuff
    LinearInterpolationBuffer * linInterpBuffer;

    //-- More internal parameter stuff
    double maxVel, tr, k;
    int encoderPulses; // default: 4096 (1024 * 4)
    int pulsesPerSample;
};

}  // namespace roboticslab

#endif  // __TECHNOSOFT_IPOS__
