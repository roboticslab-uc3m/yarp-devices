// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __TECHNOSOFT_IPOS__
#define __TECHNOSOFT_IPOS__

#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <sstream>

//#define CD_FULL_FILE  //-- Can be globally managed from father CMake. Good for debugging with polymorphism.
//#define CD_HIDE_DEBUG  //-- Can be globally managed from father CMake.
//#define CD_HIDE_SUCCESS  //-- Can be globally managed from father CMake.
//#define CD_HIDE_INFO  //-- Can be globally managed from father CMake.
//#define CD_HIDE_WARNING  //-- Can be globally managed from father CMake.
//#define CD_HIDE_ERROR  //-- Can be globally managed from father CMake.
#include "ColorDebug.hpp"
#include "ICanBusSharer.h"
#include "ITechnosoftIpos.h"

namespace roboticslab
{

/**
 * @ingroup YarpPlugins
 * \defgroup TechnosoftIpos
 * @brief Contains roboticslab::TechnosoftIpos.
 */

/**
* @ingroup TechnosoftIpos
* @brief Implementation for the Technosoft iPOS as a single CAN bus joint (controlboard raw interfaces).
*
*/
// Note: IEncodersTimedRaw inherits from IEncodersRaw
// Note: IControlLimits2Raw inherits from IControlLimitsRaw
class TechnosoftIpos : public yarp::dev::DeviceDriver, public yarp::dev::IControlLimits2Raw, public yarp::dev::IControlMode2Raw, public yarp::dev::IInteractionModeRaw, public yarp::dev::IEncodersTimedRaw,
    public yarp::dev::IPositionControl2Raw, public yarp::dev::IPositionDirectRaw, public yarp::dev::ITorqueControlRaw, public yarp::dev::IVelocityControl2Raw,
    public ICanBusSharer, public ITechnosoftIpos
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
    virtual bool setCanBusPtr(ICanBusHico *canDevicePtr);
    virtual bool setIEncodersTimedRawExternal(IEncodersTimedRaw * iEncodersTimedRaw); // -- ??
    virtual bool interpretMessage( can_msg * message);
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

    //  --------- IControlLimits2Raw Declarations. Implementation in IControlLimits2RawImpl.cpp ---------
    virtual bool setLimitsRaw(int axis, double min, double max);
    virtual bool getLimitsRaw(int axis, double *min, double *max);
    virtual bool setVelLimitsRaw(int axis, double min, double max);
    virtual bool getVelLimitsRaw(int axis, double *min, double *max);
    //-- Auxiliary functions of setLimitsRaw
    bool setMinLimitRaw(double min);
    bool setMaxLimitRaw(double max);

    //  --------- IControlModeRaw Declarations. Implementation in IControlMode2RawImpl.cpp ---------
    virtual bool setPositionModeRaw(int j);
    virtual bool setVelocityModeRaw(int j);
    virtual bool setTorqueModeRaw(int j);
    //-- Auxiliary functions (splitted) of setTorqueModeRaw
    bool setTorqueModeRaw1();
    bool setTorqueModeRaw2();
    bool setTorqueModeRaw3();

    virtual bool setImpedancePositionModeRaw(int j);
    virtual bool setImpedanceVelocityModeRaw(int j);
    virtual bool setOpenLoopModeRaw(int j);
    virtual bool getControlModeRaw(int j, int *mode);
    //-- Auxiliary functions (splitted) of getControlModeRaw
    bool getControlModeRaw1();
    bool getControlModeRaw2();
    bool getControlModeRaw3();
    bool getControlModeRaw4();

    virtual bool getControlModesRaw(int *modes);

    //  --------- IControlMode2Raw Declarations. Implementation in IControlMode2RawImpl.cpp ---------
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

    // ------- IPositionControlRaw declarations. Implementation in IPositionControl2RawImpl.cpp -------
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

    // ------- IPositionControl2Raw declarations. Implementation in IPositionControl2RawImpl.cpp ---------

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
    virtual bool setPositionsRaw(const int n_joint, const int *joints, double *refs);
    virtual bool setPositionsRaw(const double *refs);

    // -------- ITorqueControlRaw declarations. Implementation in ITorqueControlRawImpl.cpp --------
    virtual bool getRefTorquesRaw(double *t);
    virtual bool getRefTorqueRaw(int j, double *t);
    virtual bool setRefTorquesRaw(const double *t);
    virtual bool setRefTorqueRaw(int j, double t);
    virtual bool getBemfParamRaw(int j, double *bemf);
    virtual bool setBemfParamRaw(int j, double bemf);
    virtual bool setTorquePidRaw(int j, const yarp::dev::Pid &pid);
    virtual bool getTorqueRaw(int j, double *t);
    virtual bool getTorquesRaw(double *t);
    virtual bool getTorqueRangeRaw(int j, double *min, double *max);
    virtual bool getTorqueRangesRaw(double *min, double *max);
    virtual bool setTorquePidsRaw(const yarp::dev::Pid *pids);
    virtual bool setTorqueErrorLimitRaw(int j, double limit);
    virtual bool setTorqueErrorLimitsRaw(const double *limits);
    virtual bool getTorqueErrorRaw(int j, double *err);
    virtual bool getTorqueErrorsRaw(double *errs);
    virtual bool getTorquePidOutputRaw(int j, double *out);
    virtual bool getTorquePidOutputsRaw(double *outs);
    virtual bool getTorquePidRaw(int j, yarp::dev::Pid *pid);
    virtual bool getTorquePidsRaw(yarp::dev::Pid *pids);
    virtual bool getTorqueErrorLimitRaw(int j, double *limit);
    virtual bool getTorqueErrorLimitsRaw(double *limits);
    virtual bool resetTorquePidRaw(int j);
    virtual bool disableTorquePidRaw(int j);
    virtual bool enableTorquePidRaw(int j);
    virtual bool setTorqueOffsetRaw(int j, double v);

    //  --------- IVelocityControlRaw Declarations. Implementation in IVelocityControl2RawImpl.cpp ---------
    virtual bool velocityMoveRaw(int j, double sp);
    virtual bool velocityMoveRaw(const double *sp);

    //--------------IVelocityControl2Raw declarations. Implementation in IVelocityControl2RawImpl.cpp -----------------
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
    virtual bool setVelPidRaw(int j, const yarp::dev::Pid &pid);
    virtual bool setVelPidsRaw(const yarp::dev::Pid *pids);
    virtual bool getVelPidRaw(int j, yarp::dev::Pid *pid);
    virtual bool getVelPidsRaw(yarp::dev::Pid *pids);

    // ------- IInteractionModeRaw declarations. Implementation in IInteractionModeRawImpl.cpp -------
    virtual bool getInteractionModeRaw(int axis, yarp::dev::InteractionModeEnum* mode);
    virtual bool getInteractionModesRaw(int n_joints, int *joints, yarp::dev::InteractionModeEnum* modes);
    virtual bool getInteractionModesRaw(yarp::dev::InteractionModeEnum* modes);
    virtual bool setInteractionModeRaw(int axis, yarp::dev::InteractionModeEnum mode);
    virtual bool setInteractionModesRaw(int n_joints, int *joints, yarp::dev::InteractionModeEnum* modes);
    virtual bool setInteractionModesRaw(yarp::dev::InteractionModeEnum* modes);



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
    std::string msgToStr(can_msg* message);
    std::string msgToStr(uint32_t cob, uint16_t len, uint8_t * msgData);

    /** Old yarp::dev::IPositionDirectRaw implementation. */
    bool setPositionDirectModeRaw();

    int canId;
    ICanBusHico *canDevicePtr;
    double lastUsage;

    //-- Encoder stuff
    double encoder;
    uint32_t encoderTimestamp;
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
    int16_t ptModeMs;  //-- [ms]
    int ptPointCounter;
    bool ptMovementDone;
    yarp::os::Semaphore ptBuffer;

    //-- More internal parameter stuff
    double max, min, maxVel, minVel, refAcceleration, refSpeed, refTorque, refVelocity, targetPosition, tr, k;

    //-- Set the interaction mode of the robot for a set of joints, values can be stiff or compliant
    yarp::dev::InteractionModeEnum interactionMode;

    //-- Semaphores
    yarp::os::Semaphore refAccelSemaphore;
    yarp::os::Semaphore refSpeedSemaphore;
    yarp::os::Semaphore refTorqueSemaphore;
    yarp::os::Semaphore refVelocitySemaphore;
    yarp::os::Semaphore interactionModeSemaphore;
    yarp::os::Semaphore targetPositionSemaphore;




};

}  // namespace roboticslab

#endif  // __TECHNOSOFT_IPOS__

