// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __TECHNOSOFT_IPOS__
#define __TECHNOSOFT_IPOS__

#include <stdint.h>
#include <sstream>
#include <cmath>
#include <deque>

#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <yarp/dev/IControlLimits.h>
#include <yarp/dev/IRemoteVariables.h>

//#define CD_FULL_FILE  //-- Can be globally managed from father CMake. Good for debugging with polymorphism.
//#define CD_HIDE_DEBUG  //-- Can be globally managed from father CMake.
//#define CD_HIDE_SUCCESS  //-- Can be globally managed from father CMake.
//#define CD_HIDE_INFO  //-- Can be globally managed from father CMake.
//#define CD_HIDE_WARNING  //-- Can be globally managed from father CMake.
//#define CD_HIDE_ERROR  //-- Can be globally managed from father CMake.
#include "ColorDebug.h"
#include "ICanBusSharer.h"
#include "ITechnosoftIpos.h"

// https://github.com/roboticslab-uc3m/yarp-devices/issues/198#issuecomment-487279910
#define PT_BUFFER_MAX_SIZE 285
#define PVT_BUFFER_MAX_SIZE 222

namespace roboticslab
{

/**
 * @ingroup YarpPlugins
 * \defgroup TechnosoftIpos
 * @brief Contains roboticslab::TechnosoftIpos.
 */

/**
 * @ingroup YarpPlugins
 * @brief Target point in PVT interpolation mode.
 */
struct PvtPoint
{
    double p, v, t;

    static PvtPoint fromBottle(const yarp::os::Bottle & b)
    {
        PvtPoint pvtPoint;
        pvtPoint.p = b.get(0).asDouble();
        pvtPoint.v = b.get(1).asDouble();
        pvtPoint.t = b.get(2).asDouble();
        return pvtPoint;
    }

    yarp::os::Bottle toBottle() const
    {
        yarp::os::Bottle b;
        b.addDouble(p);
        b.addDouble(v);
        b.addDouble(t);
        return b;
    }
};

/**
* @ingroup TechnosoftIpos
* @brief Implementation for the Technosoft iPOS as a single CAN bus joint (controlboard raw interfaces).
*
*/
class TechnosoftIpos : public yarp::dev::DeviceDriver,
                       public yarp::dev::IControlLimitsRaw,
                       public yarp::dev::IControlModeRaw,
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
    {
        canDevicePtr = 0;
        iEncodersTimedRawExternal = 0;
    }

    //  --------- DeviceDriver Declarations. Implementation in TechnosoftIpos.cpp ---------
    virtual bool open(yarp::os::Searchable& config);
    virtual bool close();

    //  --------- ICanBusSharer Declarations. Implementation in TechnosoftIpos.cpp ---------
    virtual bool setCanBusPtr(yarp::dev::ICanBus *canDevicePtr);
    virtual bool setIEncodersTimedRawExternal(IEncodersTimedRaw * iEncodersTimedRaw); // -- ??
    virtual bool interpretMessage(const yarp::dev::CanMessage & message);
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

    //  --------- IControlLimitsRaw Declarations. Implementation in IControlLimitsRawImpl.cpp ---------
    virtual bool setLimitsRaw(int axis, double min, double max);
    virtual bool getLimitsRaw(int axis, double *min, double *max);
    virtual bool setVelLimitsRaw(int axis, double min, double max);
    virtual bool getVelLimitsRaw(int axis, double *min, double *max);
    //-- Auxiliary functions of setLimitsRaw
    bool setMinLimitRaw(double min);
    bool setMaxLimitRaw(double max);

    //  --------- IControlModeRaw Declarations. Implementation in IControlModeRawImpl.cpp ---------
    bool setPositionModeRaw(int j);
    bool setVelocityModeRaw(int j);
    bool setTorqueModeRaw(int j);
    //-- Auxiliary functions (splitted) of setTorqueModeRaw
    bool setTorqueModeRaw1();
    bool setTorqueModeRaw2();
    bool setTorqueModeRaw3();
    //-- Old yarp::dev::IPositionDirectRaw implementation
    bool setPositionDirectModeRaw();
    bool setTrajectoryModeRaw();

    virtual bool getControlModeRaw(int j, int *mode);
    //-- Auxiliary functions (splitted) of getControlModeRaw
    bool getControlModeRaw1();
    bool getControlModeRaw2();
    bool getControlModeRaw3();
    bool getControlModeRaw4();

    virtual bool getControlModesRaw(int *modes);

    virtual bool getControlModesRaw(const int n_joint, const int *joints, int *modes);
    virtual bool setControlModeRaw(const int j, const int mode);
    virtual bool setControlModesRaw(const int n_joint, const int *joints, int *modes);
    virtual bool setControlModesRaw(int *modes);

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

    // -------- ITorqueControlRaw declarations. Implementation in ITorqueControlRawImpl.cpp --------
    virtual bool getRefTorquesRaw(double *t);
    virtual bool getRefTorqueRaw(int j, double *t);
    virtual bool setRefTorquesRaw(const double *t);
    virtual bool setRefTorqueRaw(int j, double t);
    virtual bool getTorqueRaw(int j, double *t);
    virtual bool getTorquesRaw(double *t);
    virtual bool getTorqueRangeRaw(int j, double *min, double *max);
    virtual bool getTorqueRangesRaw(double *min, double *max);

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

    //  --------- Implementation in TechnosoftIpos.cpp ---------

    //-- CAN bus stuff
    /**
     * Write message to the CAN buffer.
     * @param cob Message's COB
     * @param len Data field length
     * @param msgData Data to send
     * @return true/false on success/failure.
     */
    bool send(uint32_t cob, uint16_t len, uint8_t * msgData);

    /** A helper function to display CAN messages. */
    std::string msgToStr(const yarp::dev::CanMessage & message);
    std::string msgToStr(uint32_t cob, uint16_t len, uint8_t * msgData);

    void createPvtMessage(const PvtPoint & pvtPoint, uint8_t * msg);

    int canId;
    yarp::dev::ICanBus *canDevicePtr;
    yarp::dev::ICanBufferFactory *iCanBufferFactory;
    yarp::dev::CanBuffer canOutputBuffer;
    double lastUsage;

    //-- Encoder stuff
    double encoder;
    double encoderTimestamp;
    yarp::os::Semaphore encoderReady;
    yarp::dev::IEncodersTimedRaw* iEncodersTimedRawExternal;

    //-- Mode stuff
    int getMode;
    yarp::os::Semaphore getModeReady;

    bool targetReached;
    yarp::os::Semaphore targetReachedReady;

    //-- Torque stuff
    double getTorque;
    yarp::os::Semaphore getTorqueReady;

    //-- Init stuff
    int getSwitchOn;
    yarp::os::Semaphore getSwitchOnReady;

    int getEnable;
    yarp::os::Semaphore getEnableReady;

    //-- PT stuff
    int pvtPointCounter;
    bool ptMovementDone;
    yarp::os::Semaphore ptBuffer;

    //-- More internal parameter stuff
    double max, min, maxVel, minVel, refAcceleration, refSpeed, refTorque, refVelocity, targetPosition, tr, k;
    int encoderPulses; // default: 4096 (1024 * 4)

    //-- Set the interaction mode of the robot for a set of joints, values can be stiff or compliant
    yarp::dev::InteractionModeEnum interactionMode;

    //-- Semaphores
    yarp::os::Semaphore refAccelSemaphore;
    yarp::os::Semaphore refSpeedSemaphore;
    yarp::os::Semaphore refTorqueSemaphore;
    yarp::os::Semaphore refVelocitySemaphore;
    yarp::os::Semaphore interactionModeSemaphore;
    yarp::os::Semaphore targetPositionSemaphore;

    //-- CAN output buffer
    yarp::os::Semaphore canBufferSemaphore;

    std::deque<PvtPoint> pvtQueue;
};

}  // namespace roboticslab

#endif  // __TECHNOSOFT_IPOS__

