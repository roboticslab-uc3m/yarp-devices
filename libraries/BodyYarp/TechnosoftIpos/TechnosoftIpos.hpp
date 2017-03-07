// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __TECHNOSOFT_IPOS__
#define __TECHNOSOFT_IPOS__

#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <sstream>
#include <math.h>  // roundf

//#define CD_FULL_FILE  //-- Can be globally managed from father CMake. Good for debugging with polymorphism.
//#define CD_HIDE_DEBUG  //-- Can be globally managed from father CMake.
//#define CD_HIDE_SUCCESS  //-- Can be globally managed from father CMake.
//#define CD_HIDE_INFO  //-- Can be globally managed from father CMake.
//#define CD_HIDE_WARNING  //-- Can be globally managed from father CMake.
//#define CD_HIDE_ERROR  //-- Can be globally managed from father CMake.
#include "ColorDebug.hpp"
#include "ICanBusSharer.h"
#include "ITechnosoftIpos.h"

namespace teo
{

/**
 * @ingroup BodyYarp
 * \defgroup TechnosoftIpos
 * @brief Contains teo::TechnosoftIpos.
 */

/**
* @ingroup TechnosoftIpos
* @brief Implementation for the Technosoft iPOS as a single CAN bus joint (controlboard raw interfaces).
*
*/
// Note: IEncodersTimedRaw inherits from IEncodersRaw
// Note: IControlLimits2Raw inherits from IControlLimitsRaw
class TechnosoftIpos : public yarp::dev::DeviceDriver, public yarp::dev::IControlLimits2Raw, public yarp::dev::IControlModeRaw, public yarp::dev::IInteractionModeRaw, public yarp::dev::IEncodersTimedRaw,
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

    //  --------- IControlModeRaw Declarations. Implementation in IControlModeRawImpl.cpp ---------
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

    virtual bool getControlModesRaw(int *modes)
    {
        CD_ERROR("\n");
        return false;
    }


    //  ---------- IEncodersRaw Declarations. Implementation in IEncodersRawImpl.cpp ----------
    virtual bool resetEncoderRaw(int j);
    virtual bool resetEncodersRaw()
    {
        CD_ERROR("\n");
        return false;
    }
    virtual bool setEncoderRaw(int j, double val);
    virtual bool setEncodersRaw(const double *vals)
    {
        CD_ERROR("\n");
        return false;
    }
    virtual bool getEncoderRaw(int j, double *v);
    virtual bool getEncodersRaw(double *encs)
    {
        CD_ERROR("\n");
        return false;
    }
    virtual bool getEncoderSpeedRaw(int j, double *sp);
    virtual bool getEncoderSpeedsRaw(double *spds)
    {
        CD_ERROR("\n");
        return false;
    }
    virtual bool getEncoderAccelerationRaw(int j, double *spds);
    virtual bool getEncoderAccelerationsRaw(double *accs)
    {
        CD_ERROR("\n");
        return false;
    }

    //  ---------- IEncodersTimedRaw Declarations. Implementation in IEncodersTimedRawImpl.cpp ----------
    virtual bool getEncodersTimedRaw(double *encs, double *time)
    {
        CD_ERROR("\n");
        return false;
    }
    virtual bool getEncoderTimedRaw(int j, double *encs, double *time);

    // ------- IPositionControlRaw declarations. Implementation in IPositionControlRawImpl.cpp -------
    virtual bool getAxes(int *ax)
    {
        *ax = 1;
        return true;
    }
    virtual bool setPositionModeRaw()
    {
        return setPositionModeRaw(0);
    }
    virtual bool positionMoveRaw(int j, double ref);
    virtual bool positionMoveRaw(const double *refs)
    {
        CD_ERROR("\n");
        return false;
    }
    virtual bool relativeMoveRaw(int j, double delta);
    virtual bool relativeMoveRaw(const double *deltas)
    {
        CD_ERROR("\n");
        return false;
    }
    virtual bool checkMotionDoneRaw(int j, bool *flag);
    virtual bool checkMotionDoneRaw(bool *flag)
    {
        CD_ERROR("\n");
        return false;
    }
    virtual bool setRefSpeedRaw(int j, double sp);
    virtual bool setRefSpeedsRaw(const double *spds)
    {
        CD_ERROR("\n");
        return false;
    }
    virtual bool setRefAccelerationRaw(int j, double acc);
    virtual bool setRefAccelerationsRaw(const double *accs)
    {
        CD_ERROR("\n");
        return false;
    }
    virtual bool getRefSpeedRaw(int j, double *ref);
    virtual bool getRefSpeedsRaw(double *spds)
    {
        CD_ERROR("\n");
        return false;
    }
    virtual bool getRefAccelerationRaw(int j, double *acc);
    virtual bool getRefAccelerationsRaw(double *accs)
    {
        CD_ERROR("\n");
        return false;
    }
    virtual bool stopRaw(int j);
    virtual bool stopRaw()
    {
        CD_ERROR("\n");
        return false;
    }

    // ------- IPositionControl2Raw declarations. Implementation in IPositionControl2RawImpl.cpp ---------

    /** Set new reference point for a subset of joints.
     * @param joints pointer to the array of joint numbers
     * @param refs   pointer to the array specifies the new reference points
     * @return true/false on success/failure
     */
    virtual bool positionMoveRaw(const int n_joint, const int *joints, const double *refs);

    /** Set relative position for a subset of joints.
     * @param joints pointer to the array of joint numbers
     * @param deltas pointer to the array of relative commands
     * @return true/false on success/failure
     */
    virtual bool relativeMoveRaw(const int n_joint, const int *joints, const double *deltas);

    /** Check if the current trajectory is terminated. Non blocking.
     * @param joints pointer to the array of joint numbers
     * @param flag true if the trajectory is terminated, false otherwise
     *        (a single value which is the 'and' of all joints')
     * @return true/false if network communication went well.
     */
    virtual bool checkMotionDoneRaw(const int n_joint, const int *joints, bool *flags);

    /** Set reference speed on all joints. These values are used during the
     * interpolation of the trajectory.
     * @param joints pointer to the array of joint numbers
     * @param spds   pointer to the array with speed values.
     * @return true/false upon success/failure
     */
    virtual bool setRefSpeedsRaw(const int n_joint, const int *joints, const double *spds);

    /** Set reference acceleration on all joints. This is the valure that is
     * used during the generation of the trajectory.
     * @param joints pointer to the array of joint numbers
     * @param accs   pointer to the array with acceleration values
     * @return true/false upon success/failure
     */
    virtual bool setRefAccelerationsRaw(const int n_joint, const int *joints, const double *accs);

    /** Get reference speed of all joints. These are the  values used during the
     * interpolation of the trajectory.
     * @param joints pointer to the array of joint numbers
     * @param spds   pointer to the array that will store the speed values.
     * @return true/false upon success/failure
     */
    virtual bool getRefSpeedsRaw(const int n_joint, const int *joints, double *spds);

    /** Get reference acceleration for a joint. Returns the acceleration used to
     * generate the trajectory profile.
     * @param joints pointer to the array of joint numbers
     * @param accs   pointer to the array that will store the acceleration values
     * @return true/false on success/failure
     */
    virtual bool getRefAccelerationsRaw(const int n_joint, const int *joints, double *accs);

    /** Stop motion for subset of joints
     * @param joints pointer to the array of joint numbers
     * @return true/false on success/failure
     */
    virtual bool stopRaw(const int n_joint, const int *joints);

    /** Get the last position reference for the specified axis.
     *  This is the dual of PositionMove and shall return only values sent using
     *  IPositionControl interface.
     *  If other interfaces like IPositionDirect are implemented by the device, this call
     *  must ignore their values, i.e. this call must never return a reference sent using
     *  IPositionDirect::SetPosition
     * @param ref last reference sent using PositionMove functions
     * @return true/false on success/failure
     */
    virtual bool getTargetPositionRaw(const int joint, double *ref);

    /** Get the last position reference for all axes.
     *  This is the dual of PositionMove and shall return only values sent using
     *  IPositionControl interface.
     *  If other interfaces like IPositionDirect are implemented by the device, this call
     *  must ignore their values, i.e. this call must never return a reference sent using
     *  IPositionDirect::SetPosition
     * @param ref last reference sent using PositionMove functions
     * @return true/false on success/failure
     */
    virtual bool getTargetPositionsRaw(double *refs);

    /** Get the last position reference for the specified group of axes.
     *  This is the dual of PositionMove and shall return only values sent using
     *  IPositionControl interface.
     *  If other interfaces like IPositionDirect are implemented by the device, this call
     *  must ignore their values, i.e. this call must never return a reference sent using
     *  IPositionDirect::SetPosition
     * @param ref last reference sent using PositionMove functions
     * @return true/false on success/failure
     */
    virtual bool getTargetPositionsRaw(const int n_joint, const int *joints, double *refs);

    // ------- IPositionDirectRaw declarations. Implementation in IPositionDirectRawImpl.cpp -------
    virtual bool setPositionDirectModeRaw();
    virtual bool setPositionRaw(int j, double ref);
    virtual bool setPositionsRaw(const int n_joint, const int *joints, double *refs);
    virtual bool setPositionsRaw(const double *refs);

    // -------- ITorqueControlRaw declarations. Implementation in ITorqueControlRawImpl.cpp --------
    virtual bool setTorqueModeRaw()
    {
        CD_ERROR("\n");
        return false;
    }
    virtual bool getRefTorquesRaw(double *t)
    {
        CD_ERROR("\n");
        return false;
    }
    virtual bool getRefTorqueRaw(int j, double *t);
    virtual bool setRefTorquesRaw(const double *t)
    {
        CD_ERROR("\n");
        return false;
    }
    virtual bool setRefTorqueRaw(int j, double t);
    virtual bool getBemfParamRaw(int j, double *bemf);
    virtual bool setBemfParamRaw(int j, double bemf);
    virtual bool setTorquePidRaw(int j, const yarp::dev::Pid &pid);
    virtual bool getTorqueRaw(int j, double *t);
    virtual bool getTorquesRaw(double *t)
    {
        CD_ERROR("\n");
        return false;
    }
    virtual bool getTorqueRangeRaw(int j, double *min, double *max);
    virtual bool getTorqueRangesRaw(double *min, double *max)
    {
        CD_ERROR("\n");
        return false;
    }
    virtual bool setTorquePidsRaw(const yarp::dev::Pid *pids)
    {
        CD_ERROR("\n");
        return false;
    }
    virtual bool setTorqueErrorLimitRaw(int j, double limit);
    virtual bool setTorqueErrorLimitsRaw(const double *limits)
    {
        CD_ERROR("\n");
        return false;
    }
    virtual bool getTorqueErrorRaw(int j, double *err);
    virtual bool getTorqueErrorsRaw(double *errs)
    {
        CD_ERROR("\n");
        return false;
    }
    virtual bool getTorquePidOutputRaw(int j, double *out);
    virtual bool getTorquePidOutputsRaw(double *outs)
    {
        CD_ERROR("\n");
        return false;
    }
    virtual bool getTorquePidRaw(int j, yarp::dev::Pid *pid);
    virtual bool getTorquePidsRaw(yarp::dev::Pid *pids)
    {
        CD_ERROR("\n");
        return false;
    }
    virtual bool getTorqueErrorLimitRaw(int j, double *limit);
    virtual bool getTorqueErrorLimitsRaw(double *limits)
    {
        CD_ERROR("\n");
        return false;
    }
    virtual bool resetTorquePidRaw(int j);
    virtual bool disableTorquePidRaw(int j);
    virtual bool enableTorquePidRaw(int j);
    virtual bool setTorqueOffsetRaw(int j, double v);

    //  --------- IVelocityControl Declarations. Implementation in IVelocityControlImpl.cpp ---------
    virtual bool setVelocityModeRaw()
    {
        CD_ERROR("\n");
        return false;
    }
    virtual bool velocityMoveRaw(int j, double sp);
    virtual bool velocityMoveRaw(const double *sp)
    {
        CD_ERROR("\n");
        return false;
    }

    // ------- IInteractionModeRaw declarations. Implementation in IInteractionModeRawImpl.cpp -------

    /**
     * Get the current interaction mode of the robot, values can be stiff or compliant.
     * @param axis joint number
     * @param mode contains the requested information about interaction mode of the joint
     * @return true or false on success or failure.
     */
    virtual bool getInteractionModeRaw(int axis, yarp::dev::InteractionModeEnum* mode);


    /**
     * Get the current interaction mode of the robot for a set of joints, values can be stiff or compliant.
     * @param n_joints how many joints this command is referring to
     * @param joints list of joints controlled. The size of this array is n_joints
     * @param modes array containing the requested information about interaction mode, one value for each joint, the size is n_joints.
     *          for example:
     *          n_joint  3
     *          joints   0  2  4
     *          refs    VOCAB_IM_STIFF VOCAB_IM_STIFF VOCAB_IM_COMPLIANT
     * @return true or false on success or failure.
     */
    virtual bool getInteractionModesRaw(int n_joints, int *joints, yarp::dev::InteractionModeEnum* modes);


    /**
     * Get the current interaction mode of the robot for a all the joints, values can be stiff or compliant.
     * @param mode array containing the requested information about interaction mode, one value for each joint.
     * @return true or false on success or failure.
     */
    virtual bool getInteractionModesRaw(yarp::dev::InteractionModeEnum* modes);


    /**
     * Set the interaction mode of the robot, values can be stiff or compliant.
     * Please note that some robot may not implement certain types of interaction, so always check the return value.
     * @param axis joint number
     * @param mode the desired interaction mode
     * @return true or false on success or failure.
     */
    virtual bool setInteractionModeRaw(int axis, yarp::dev::InteractionModeEnum mode);


    /**
     * Set the interaction mode of the robot for a set of joints, values can be stiff or compliant.
     * Please note that some robot may not implement certain types of interaction, so always check the return value.
     * @param n_joints how many joints this command is referring to
     * @param joints list of joints controlled. The size of this array is n_joints
     * @param modes array containing the desired interaction mode, one value for each joint, the size is n_joints.
     *          for example:
     *          n_joint  3
     *          joints   0  2  4
     *          refs    VOCAB_IM_STIFF VOCAB_IM_STIFF VOCAB_IM_COMPLIANT
     * @return true or false on success or failure. If one or more joint fails, the return value will be false.
     */
    virtual bool setInteractionModesRaw(int n_joints, int *joints, yarp::dev::InteractionModeEnum* modes);

    /**
     * Set the interaction mode of the robot for a all the joints, values can be stiff or compliant.
     * Some robot may not implement some types of interaction, so always check the return value
     * @param mode array with the desired interaction mode for all joints, length is the total number of joints for the part
     * @return true or false on success or failure. If one or more joint fails, the return value will be false.
     */
    virtual bool setInteractionModesRaw(yarp::dev::InteractionModeEnum* modes);

    //--------------IVelocityControl2Raw declarations. Implementation in IVelocityControl2RawImpl.cpp -----------------
    /**
     * Start motion at a given speed for a subset of joints.
     * @param n_joint how many joints this command is referring to
     * @param joints pointer to the array of joint numbers
     * @param spds    pointer to the array containing the new speed values
     * @return true/false upon success/failure
     */
    virtual bool velocityMoveRaw(const int n_joint, const int *joints, const double *spds);

     /** Get the last reference speed set by velocityMove for single joint.
     * @param j joint number
     * @param vel returns the requested reference.
     * @return true/false on success/failure
     */
    virtual bool getRefVelocityRaw(const int joint, double *vel);

    /** Get the last reference speed set by velocityMove for all joints.
     * @param vels pointer to the array containing the new speed values, one value for each joint
     * @return true/false on success/failure
     */
    virtual bool getRefVelocitiesRaw(double *vels);

    /** Get the last reference speed set by velocityMove for a group of joints.
     * @param n_joint how many joints this command is referring to
     * @param joints of joints controlled. The size of this array is n_joints
     * @param vels pointer to the array containing the requested values, one value for each joint.
     *  The size of the array is n_joints.
     * @return true/false on success/failure
     */
    virtual bool getRefVelocitiesRaw(const int n_joint, const int *joints, double *vels);

    /** Set reference acceleration for a subset of joints. This is the valure that is
     * used during the generation of the trajectory.
     * @param joints pointer to the array of joint numbers
     * @param accs   pointer to the array containing acceleration values
     * @return true/false upon success/failure
     */
    // ------------------- Just declareted in IPositionControl2Raw
    // -- virtual bool setRefAccelerationsRaw(const int n_joint, const int *joints, const double *accs);

    /** Get reference acceleration for a subset of joints. These are the values used during the
     * interpolation of the trajectory.
     * @param joints pointer to the array of joint numbers
     * @param accs   pointer to the array that will store the acceleration values.
     * @return true/false on success or failure
     */
    // ------------------- Just declareted in IPositionControl2Raw
    // -- virtual bool getRefAccelerationsRaw(const int n_joint, const int *joints, double *accs);

    /** Stop motion for a subset of joints
     * @param joints pointer to the array of joint numbers
     * @return true/false on success or failure
     */
    // ------------------- Just declareted in IPositionControl2Raw
    // -- virtual bool stopRaw(const int n_joint, const int *joints);

    /** Set new velocity pid value for a joint
     * @param j joint number
     * @param pid new pid value
     * @return true/false on success/failure
     */
    virtual bool setVelPidRaw(int j, const yarp::dev::Pid &pid);

    /** Set new velocity pid value on multiple joints
     * @param pids pointer to a vector of pids
     * @return true/false upon success/failure
     */
    virtual bool setVelPidsRaw(const yarp::dev::Pid *pids);

    /** Get current velocity pid value for a specific joint.
     * @param j joint number
     * @param pid pointer to storage for the return value.
     * @return success/failure
     */
    virtual bool getVelPidRaw(int j, yarp::dev::Pid *pid);

    /** Get current velocity pid value for a specific subset of joints.
     * @param pids vector that will store the values of the pids.
     * @return success/failure
     */
    virtual bool getVelPidsRaw(yarp::dev::Pid *pids);

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

}  // namespace teo

#endif  // __TECHNOSOFT_IPOS__

