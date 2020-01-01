// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __AMOR_CONTROLBOARD_HPP__
#define __AMOR_CONTROLBOARD_HPP__

#include <stdio.h>
#include <vector>

#include <yarp/os/all.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>

#include <amor.h>

#include "ColorDebug.h"

#define DEFAULT_CAN_LIBRARY "libeddriver.so"
#define DEFAULT_CAN_PORT 0

namespace roboticslab
{

/**
 * @ingroup YarpPlugins
 * @defgroup AmorControlboard
 * @brief Contains roboticslab::AmorControlboard.
 */

/**
* @ingroup AmorControlboard
* @brief Implements several yarp::dev:: controlboard interfaces.
*/
class AmorControlboard : public yarp::dev::DeviceDriver,
                         public yarp::dev::IAxisInfo,
                         public yarp::dev::IControlLimits,
                         public yarp::dev::IControlMode,
                         public yarp::dev::ICurrentControl,
                         public yarp::dev::IEncodersTimed,
                         public yarp::dev::IInteractionMode,
                         public yarp::dev::IPositionControl,
                         public yarp::dev::IRemoteVariables,
                         public yarp::dev::ITorqueControl,
                         public yarp::dev::IVelocityControl
{
public:

    AmorControlboard() : handle(AMOR_INVALID_HANDLE),
                         usingCartesianController(false),
                         controlMode(VOCAB_CM_POSITION)
    { }

    ~AmorControlboard()
    { close(); }

    // ------- IPositionControl declarations. Implementation in IPositionControlImpl.cpp -------

    /**
     * Get the number of controlled axes. This command asks the number of controlled
     * axes for the current physical interface.
     * @param ax pointer to storage
     * @return true/false.
     */
    virtual bool getAxes(int *ax);

    /** Set new reference point for a single axis.
     * @param j joint number
     * @param ref specifies the new ref point
     * @return true/false on success/failure
     */
    virtual bool positionMove(int j, double ref);

    /** Set new reference point for all axes.
     * @param refs array, new reference points.
     * @return true/false on success/failure
     */
    virtual bool positionMove(const double *refs);

    /** Set relative position. The command is relative to the
     * current position of the axis.
     * @param j joint axis number
     * @param delta relative command
     * @return true/false on success/failure
     */
    virtual bool relativeMove(int j, double delta);

    /** Set relative position, all joints.
     * @param deltas pointer to the relative commands
     * @return true/false on success/failure
     */
    virtual bool relativeMove(const double *deltas);

    /** Check if the current trajectory is terminated. Non blocking.
     * @return true if the trajectory is terminated, false otherwise
     */
    virtual bool checkMotionDone(int j, bool *flag);

    /** Check if the current trajectory is terminated. Non blocking.
     * @return true if the trajectory is terminated, false otherwise
     */
    virtual bool checkMotionDone(bool *flag);

    /** Set reference speed for a joint, this is the speed used during the
     * interpolation of the trajectory.
     * @param j joint number
     * @param sp speed value
     * @return true/false upon success/failure
     */
    virtual bool setRefSpeed(int j, double sp);

    /** Set reference speed on all joints. These values are used during the
     * interpolation of the trajectory.
     * @param spds pointer to the array of speed values.
     * @return true/false upon success/failure
     */
    virtual bool setRefSpeeds(const double *spds);

    /** Set reference acceleration for a joint. This value is used during the
     * trajectory generation.
     * @param j joint number
     * @param acc acceleration value
     * @return true/false upon success/failure
     */
    virtual bool setRefAcceleration(int j, double acc);

    /** Set reference acceleration on all joints. This is the valure that is
     * used during the generation of the trajectory.
     * @param accs pointer to the array of acceleration values
     * @return true/false upon success/failure
     */
    virtual bool setRefAccelerations(const double *accs);

    /** Get reference speed for a joint. Returns the speed used to
     * generate the trajectory profile.
     * @param j joint number
     * @param ref pointer to storage for the return value
     * @return true/false on success or failure
     */
    virtual bool getRefSpeed(int j, double *ref);

    /** Get reference speed of all joints. These are the  values used during the
     * interpolation of the trajectory.
     * @param spds pointer to the array that will store the speed values.
     */
    virtual bool getRefSpeeds(double *spds);

    /** Get reference acceleration for a joint. Returns the acceleration used to
     * generate the trajectory profile.
     * @param j joint number
     * @param acc pointer to storage for the return value
     * @return true/false on success/failure
     */
    virtual bool getRefAcceleration(int j, double *acc);

    /** Get reference acceleration of all joints. These are the values used during the
     * interpolation of the trajectory.
     * @param accs pointer to the array that will store the acceleration values.
     * @return true/false on success or failure
     */
    virtual bool getRefAccelerations(double *accs);

    /** Stop motion, single joint
     * @param j joint number
     * @return true/false on success/failure
     */
    virtual bool stop(int j);

    /** Stop motion, multiple joints
     * @return true/false on success/failure
     */
    virtual bool stop();

    /** Set new reference point for a subset of joints.
     * @param joints pointer to the array of joint numbers
     * @param refs   pointer to the array specifing the new reference points
     * @return true/false on success/failure
     */
    virtual bool positionMove(const int n_joint, const int *joints, const double *refs);

    /** Set relative position for a subset of joints.
     * @param joints pointer to the array of joint numbers
     * @param deltas pointer to the array of relative commands
     * @return true/false on success/failure
     */
    virtual bool relativeMove(const int n_joint, const int *joints, const double *deltas);

    /** Check if the current trajectory is terminated. Non blocking.
     * @param joints pointer to the array of joint numbers
     * @param flags  pointer to return value (logical "and" of all set of joints)
     * @return true/false if network communication went well.
     */
    virtual bool checkMotionDone(const int n_joint, const int *joints, bool *flags);

    /** Set reference speed on all joints. These values are used during the
     * interpolation of the trajectory.
     * @param joints pointer to the array of joint numbers
     * @param spds   pointer to the array with speed values.
     * @return true/false upon success/failure
     */
    virtual bool setRefSpeeds(const int n_joint, const int *joints, const double *spds);

    /** Set reference acceleration on all joints. This is the valure that is
     * used during the generation of the trajectory.
     * @param joints pointer to the array of joint numbers
     * @param accs   pointer to the array with acceleration values
     * @return true/false upon success/failure
     */
    virtual bool setRefAccelerations(const int n_joint, const int *joints, const double *accs);

    /** Get reference speed of all joints. These are the  values used during the
     * interpolation of the trajectory.
     * @param joints pointer to the array of joint numbers
     * @param spds   pointer to the array that will store the speed values.
     * @return true/false upon success/failure
     */
    virtual bool getRefSpeeds(const int n_joint, const int *joints, double *spds);

    /** Get reference acceleration for a joint. Returns the acceleration used to
     * generate the trajectory profile.
     * @param joints pointer to the array of joint numbers
     * @param accs   pointer to the array that will store the acceleration values
     * @return true/false on success/failure
     */
    virtual bool getRefAccelerations(const int n_joint, const int *joints, double *accs);

    /** Stop motion for subset of joints
     * @param joints pointer to the array of joint numbers
     * @return true/false on success/failure
     */
    virtual bool stop(const int n_joint, const int *joints);

    /** Get the last position reference for the specified axis.
     *  This is the dual of PositionMove and shall return only values sent using
     *  IPositionControl interface.
     *  If other interfaces like IPositionDirect are implemented by the device, this call
     *  must ignore their values, i.e. this call must never return a reference sent using
     *  IPositionDirect::SetPosition
     * @param ref last reference sent using PositionMove functions
     * @return true/false on success/failure
     */
    virtual bool getTargetPosition(const int joint, double *ref);

    /** Get the last position reference for all axes.
     *  This is the dual of PositionMove and shall return only values sent using
     *  IPositionControl interface.
     *  If other interfaces like IPositionDirect are implemented by the device, this call
     *  must ignore their values, i.e. this call must never return a reference sent using
     *  IPositionDirect::SetPosition
     * @param ref last reference sent using PositionMove functions
     * @return true/false on success/failure
     */
    virtual bool getTargetPositions(double *refs);

    /** Get the last position reference for the specified group of axes.
     *  This is the dual of PositionMove and shall return only values sent using
     *  IPositionControl interface.
     *  If other interfaces like IPositionDirect are implemented by the device, this call
     *  must ignore their values, i.e. this call must never return a reference sent using
     *  IPositionDirect::SetPosition
     * @param ref last reference sent using PositionMove functions
     * @return true/false on success/failure
     */
    virtual bool getTargetPositions(const int n_joint, const int *joints, double *refs);

//  ---------- IEncoders declarations. Implementation in IEncodersImpl.cpp ----------

    /**
     * Reset encoder, single joint. Set the encoder value to zero
     * @param j encoder number
     * @return true/false
     */
    virtual bool resetEncoder(int j);

    /**
     * Reset encoders. Set the encoders value to zero
     * @return true/false
     */
    virtual bool resetEncoders();

    /**
     * Set the value of the encoder for a given joint.
     * @param j encoder number
     * @param val new value
     * @return true/false
     */
    virtual bool setEncoder(int j, double val);

    /**
     * Set the value of all encoders.
     * @param vals pointer to the new values
     * @return true/false
     */
    virtual bool setEncoders(const double *vals);

    /**
     * Read the value of an encoder.
     * @param j encoder number
     * @param v pointer to storage for the return value
     * @return true/false, upon success/failure (you knew it, uh?)
     */
    virtual bool getEncoder(int j, double *v);

    /**
     * Read the position of all axes.
     * @param encs pointer to the array that will contain the output
     * @return true/false on success/failure
     */
    virtual bool getEncoders(double *encs);

    /**
     * Read the instantaneous speed of an axis.
     * @param j axis number
     * @param sp pointer to storage for the output
     * @return true if successful, false ... otherwise.
     */
    virtual bool getEncoderSpeed(int j, double *sp);

    /**
     * Read the instantaneous speed of all axes.
     * @param spds pointer to storage for the output values
     * @return guess what? (true/false on success or failure).
     */
    virtual bool getEncoderSpeeds(double *spds);

    /**
     * Read the instantaneous acceleration of an axis.
     * @param j axis number
     * @param spds pointer to the array that will contain the output
     */
    virtual bool getEncoderAcceleration(int j, double *spds);

    /**
     * Read the instantaneous acceleration of all axes.
     * @param accs pointer to the array that will contain the output
     * @return true if all goes well, false if anything bad happens.
     */
    virtual bool getEncoderAccelerations(double *accs);

//  --------- IEncodersTimed declarations. Implementation in IEncodersTimedImpl.cpp ---------

    /**
    * Read the instantaneous acceleration of all axes.
    * \param encs pointer to the array that will contain the output
    * \param time pointer to the array that will contain individual timestamps
    * \return true if all goes well, false if anything bad happens.
    */
   virtual bool getEncodersTimed(double *encs, double *time);

   /**
   * Read the instantaneous acceleration of all axes.
   * \param j axis index
   * \param encs encoder value (pointer to)
   * \param time corresponding timestamp (pointer to)
   * \return true if all goes well, false if anything bad happens.
   */
   virtual bool getEncoderTimed(int j, double *encs, double *time);

//  --------- IVelocityControl Declarations. Implementation in IVelocityControlImpl.cpp ---------

    /**
     * Start motion at a given speed, single joint.
     * @param j joint number
     * @param sp speed value
     * @return bool/false upone success/failure
     */
    virtual bool velocityMove(int j, double sp);

    /**
     * Start motion at a given speed, multiple joints.
     * @param sp pointer to the array containing the new speed values
     * @return true/false upon success/failure
     */
    virtual bool velocityMove(const double *sp);

    /** Start motion at a given speed for a subset of joints.
     * @param n_joint how many joints this command is referring to
     * @param joints of joints controlled. The size of this array is n_joints
     * @param spds pointer to the array containing the new speed values, one value for each joint, the size of the array is n_joints.
     * The first value will be the new reference fot the joint joints[0].
     *          for example:
     *          n_joint  3
     *          joints   0  2  4
     *          spds    10 30 40
     * @return true/false on success/failure
     */
    virtual bool velocityMove(const int n_joint, const int *joints, const double *spds);

    /** Get the last reference speed set by velocityMove for single joint.
     * @param j joint number
     * @param vel returns the requested reference.
     * @return true/false on success/failure
     */
    virtual bool getRefVelocity(const int joint, double *vel);

    /** Get the last reference speed set by velocityMove for all joints.
     * @param vels pointer to the array containing the new speed values, one value for each joint
     * @return true/false on success/failure
     */
    virtual bool getRefVelocities(double *vels);

    /** Get the last reference speed set by velocityMove for a group of joints.
     * @param n_joint how many joints this command is referring to
     * @param joints of joints controlled. The size of this array is n_joints
     * @param vels pointer to the array containing the requested values, one value for each joint.
     *  The size of the array is n_joints.
     * @return true/false on success/failure
     */
    virtual bool getRefVelocities(const int n_joint, const int *joints, double *vels);

//  --------- IControlLimits declarations. Implementation in IControlLimitsImpl.cpp ---------

    /**
     * Set the software limits for a particular axis, the behavior of the
     * control card when these limits are exceeded, depends on the implementation.
     * @param axis joint number (why am I telling you this)
     * @param min the value of the lower limit
     * @param max the value of the upper limit
     * @return true or false on success or failure
     */
    virtual bool setLimits(int axis, double min, double max);

    /** Get the software limits for a particular axis.
     * @param axis joint number (again... why am I telling you this)
     * @param pointer to store the value of the lower limit
     * @param pointer to store the value of the upper limit
     * @return true if everything goes fine, false otherwise.
     */
    virtual bool getLimits(int axis, double *min, double *max);

    /**
     * Set the software speed limits for a particular axis, the behavior of the
     * control card when these limits are exceeded, depends on the implementation.
     * @param axis joint number
     * @param min the value of the lower limit
     * @param max the value of the upper limit
     * @return true or false on success or failure
     */
    virtual bool setVelLimits(int axis, double min, double max);

    /**
     * Get the software speed limits for a particular axis.
     * @param axis joint number
     * @param min pointer to store the value of the lower limit
     * @param max pointer to store the value of the upper limit
     * @return true if everything goes fine, false otherwise.
     */
    virtual bool getVelLimits(int axis, double *min, double *max);

//  --------- IControlMode declarations. Implementation in IControlModeImpl.cpp ---------

    /**
    * Get the current control mode.
    * @param j: joint number
    * @param mode: a vocab of the current control mode for joint j.
    * @return: true/false success failure.
    */
    virtual bool getControlMode(int j, int *mode);

    /**
    * Get the current control mode (multiple joints).
    * @param modes: a vector containing vocabs for the current control modes of the joints.
    * @return: true/false success failure.
    */
    virtual bool getControlModes(int *modes);

    /**
    * Get the current control mode for a subset of axes.
    * @param n_joints how many joints this command is referring to
    * @param joints list of joint numbers, the size of this array is n_joints
    * @param modes array containing the new controlmodes, one value for each joint, the size is n_joints.
    *          The first value will be the new reference fot the joint joints[0].
    *          for example:
    *          n_joint  3
    *          joints   0  2  4
    *          modes    VOCAB_CM_POSITION VOCAB_CM_VELOCITY VOCAB_CM_POSITION
    * @return true/false success failure.
    */
    virtual bool getControlModes(const int n_joint, const int *joints, int *modes);

    /**
    * Set the current control mode.
    * @param j: joint number
    * @param mode: a vocab of the desired control mode for joint j.
    * @return true if the new controlMode was successfully set, false if the message was not received or
    *         the joint was unable to switch to the desired controlMode
    *         (e.g. the joint is on a fault condition or the desired mode is not implemented).    */
    virtual bool setControlMode(const int j, const int mode);

    /**
    * Set the current control mode for a subset of axes.
    * @param n_joints how many joints this command is referring to
    * @param joints list of joint numbers, the size of this array is n_joints
    * @param modes array containing the new controlmodes, one value for each joint, the size is n_joints.
    *          The first value will be the new reference fot the joint joints[0].
    *          for example:
    *          n_joint  3
    *          joints   0  2  4
    *          modes    VOCAB_CM_POSITION VOCAB_CM_VELOCITY VOCAB_CM_POSITION
    * @return true if the new controlMode was successfully set, false if the message was not received or
    *         the joint was unable to switch to the desired controlMode
    *         (e.g. the joint is on a fault condition or the desired mode is not implemented).
    */
    virtual bool setControlModes(const int n_joint, const int *joints, int *modes);

    /**
    * Set the current control mode (multiple joints).
    * @param modes: a vector containing vocabs for the desired control modes of the joints.
    * @return true if the new controlMode was successfully set, false if the message was not received or
    *         the joint was unable to switch to the desired controlMode
    *         (e.g. the joint is on a fault condition or the desired mode is not implemented).
    */
    virtual bool setControlModes(int *modes);

// -------- IAxisInfo declarations. Implementation in IAxisInfoImpl.cpp --------

    /**
     * Get the name for a particular axis.
     * @param axis joint number
     * @param name the axis name
     * @return true if everything goes fine, false otherwise.
     */
    virtual bool getAxisName(int axis, std::string& name);

    /**
     * Get the joint type (e.g. revolute/prismatic) for a particular axis.
     * @param axis joint number
     * @param type the joint type
     * @return true if everything goes fine, false otherwise.
     */
    virtual bool getJointType(int axis, yarp::dev::JointTypeEnum& type);

// -------- ITorqueControl declarations. Implementation in ITorqueControlImpl.cpp --------

    /** Get the reference value of the torque for all joints.
     * This is NOT the feedback (see getTorques instead).
     * @param t pointer to the array of torque values
     * @return true/false on success/failure
     */
    virtual bool getRefTorques(double *t);

    /** Get the reference value of the torque for a given joint.
     * This is NOT the feedback (see getTorque instead).
     * @param j joint number
     * @param t the returned reference torque of joint j
     * @return true/false on success/failure
     */
    virtual bool getRefTorque(int j, double *t);

    /** Set the reference value of the torque for all joints.
     * @param t pointer to the array of torque values
     * @return true/false on success/failure
     */
    virtual bool setRefTorques(const double *t);

    /** Set the reference value of the torque for a given joint.
     * @param j joint number
     * @param t new value
     * @return true/false on success/failure
     */
    virtual bool setRefTorque(int j, double t);

    /** Set new torque reference for a subset of joints.
     * @param joints pointer to the array of joint numbers
     * @param refs   pointer to the array specifing the new torque reference
     * @return true/false on success/failure
     */
    virtual bool setRefTorques(const int n_joint, const int *joints, const double *t);

    /** Get a subset of motor parameters (bemf, ktau etc) useful for torque control.
     * @param j joint number
     * @param params a struct containing the motor parameters to be retrieved
     * @return true/false on success/failure
     */
    virtual bool getMotorTorqueParams(int j, yarp::dev::MotorTorqueParameters *params);

    /** Set a subset of motor parameters (bemf, ktau etc) useful for torque control.
     * @param j joint number
     * @param params a struct containing the motor parameters to be set
     * @return true/false on success/failure
     */
    virtual bool setMotorTorqueParams(int j, const yarp::dev::MotorTorqueParameters params);

    /** Get the value of the torque on a given joint (this is the
     * feedback if you have a torque sensor).
     * @param j joint number
     * @param t pointer to the result value
     * @return true/false on success/failure
     */
    virtual bool getTorque(int j, double *t);

    /** Get the value of the torque for all joints (this is
     * the feedback if you have torque sensors).
     * @param t pointer to the array that will store the output
     * @return true/false on success/failure
     */
    virtual bool getTorques(double *t);

    /** Get the full scale of the torque sensor of a given joint
     * @param j joint number
     * @param min minimum torque of the joint j
     * @param max maximum torque of the joint j
     * @return true/false on success/failure
     */
    virtual bool getTorqueRange(int j, double *min, double *max);

    /** Get the full scale of the torque sensors of all joints
     * @param min pointer to the array that will store minimum torques of the joints
     * @param max pointer to the array that will store maximum torques of the joints
     * @return true/false on success/failure
     */
    virtual bool getTorqueRanges(double *min, double *max);

// -------- IInteractionMode declarations. Implementation in IInteractionModeImpl.cpp --------

    /**
     * Get the current interaction mode of the robot, values can be stiff or compliant.
     * @param axis joint number
     * @param mode contains the requested information about interaction mode of the joint
     * @return true or false on success or failure.
     */
    virtual bool getInteractionMode(int axis, yarp::dev::InteractionModeEnum* mode);

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
    virtual bool getInteractionModes(int n_joints, int *joints, yarp::dev::InteractionModeEnum* modes);

    /**
     * Get the current interaction mode of the robot for a all the joints, values can be stiff or compliant.
     * @param mode array containing the requested information about interaction mode, one value for each joint.
     * @return true or false on success or failure.
     */
    virtual bool getInteractionModes(yarp::dev::InteractionModeEnum* modes);

    /**
     * Set the interaction mode of the robot, values can be stiff or compliant.
     * Please note that some robot may not implement certain types of interaction, so always check the return value.
     * @param axis joint number
     * @param mode the desired interaction mode
     * @return true or false on success or failure.
     */
    virtual bool setInteractionMode(int axis, yarp::dev::InteractionModeEnum mode);

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
    virtual bool setInteractionModes(int n_joints, int *joints, yarp::dev::InteractionModeEnum* modes);

    /**
     * Set the interaction mode of the robot for a all the joints, values can be stiff or compliant.
     * Some robot may not implement some types of interaction, so always check the return value
     * @param mode array with the desired interaction mode for all joints, length is the total number of joints for the part
     * @return true or false on success or failure. If one or more joint fails, the return value will be false.
     */
    virtual bool setInteractionModes(yarp::dev::InteractionModeEnum* modes);

//  --------- ICurrentControl Declarations. Implementation in ICurrentControlImpl.cpp ---------

    /**
     * Retrieves the number of controlled axes from the current physical interface.
     * @param ax returns the number of controlled axes.
     * @return true/false on success/failure
     */
    virtual bool getNumberOfMotors(int *ax);

    /** Get the instantaneous current measurement for a single motor
    * @param m motor number
    * @param curr pointer to the result value. Value is expressed in amperes.
    * @return true/false on success/failure
    */
    virtual bool getCurrent(int m, double *curr);

    /** Get the instantaneous current measurement for all motors
    * @param currs pointer to the array that will store the output. Values are expressed in amperes.
    * @return true/false on success/failure
    */
    virtual bool getCurrents(double *currs);

    /** Get the full scale of the current measurement for a given motor (e.g. -20A +20A)
    * Reference values set by user with methods such as setRefCurrent() should be in this range.
    * This method is not related to the current overload protection methods belonging to the iAmplifierControl interface.
    * @param m motor number
    * @param min minimum current of the motor m
    * @param max maximum current of the motor m
    * @return true/false on success/failure
    */
    virtual bool getCurrentRange(int m, double *min, double *max);

    /** Get the full scale of the current measurements for all motors motor (e.g. -20A +20A)
    * Reference values set by user with methods such as setRefCurrent() should be in this range.
    * This method is not related to the current overload protection methods belonging to the iAmplifierControl interface.
    * @param min pointer to the array that will store minimum currents
    * @param max pointer to the array that will store maximum currents
    * @return true/false on success/failure
    */
    virtual bool getCurrentRanges(double *min, double *max);

    /** Set the reference value of the currents for all motors.
    * @param currs the array containing the reference current values. Values are expressed in amperes.
    * @return true/false on success/failure
    */
    virtual bool setRefCurrents(const double *currs);

    /** Set the reference value of the current for a single motor.
    * @param m motor number
    * @param curr the current reference value for motor m. Value is expressed in amperes.
    * @return true/false on success/failure
    */
    virtual bool setRefCurrent(int m, double curr);

    /**  Set the reference value of the current for a group of motors.
    * @param n_motor size of motors ans currs arrays
    * @param motors  pointer to the array containing the list of motor numbers
    * @param currs   pointer to the array specifying the new current references
    * @return true/false on success/failure
    */
    virtual bool setRefCurrents(const int n_motor, const int *motors, const double *currs);

   /** Get the reference value of the currents for all motors.
     * @param currs pointer to the array to be filled with reference current values. Values are expressed in amperes.
     * @return true/false on success/failure
     */
    virtual bool getRefCurrents(double *currs);

    /** Get the reference value of the current for a single motor.
    * @param m motor number
    * @param curr the current reference value for motor m. Value is expressed in amperes.
    * @return true/false on success/failure
    */
    virtual bool getRefCurrent(int m, double *curr);

// -----------IRemoteVariables Declarations. Implementation in IRemoteVariablesImpl.cpp --------------

    virtual bool getRemoteVariable(std::string key, yarp::os::Bottle& val);

    virtual bool setRemoteVariable(std::string key, const yarp::os::Bottle& val);

    virtual bool getRemoteVariablesList(yarp::os::Bottle* listOfKeys);

// -------- DeviceDriver declarations. Implementation in IDeviceDriverImpl.cpp --------

    /**
     * Open the DeviceDriver.
     * @param config is a list of parameters for the device.
     * Which parameters are effective for your device can vary.
     * See \ref dev_examples "device invocation examples".
     * If there is no example for your device,
     * you can run the "yarpdev" program with the verbose flag
     * set to probe what parameters the device is checking.
     * If that fails too,
     * you'll need to read the source code (please nag one of the
     * yarp developers to add documentation for your device).
     * @return true/false upon success/failure
     */
    virtual bool open(yarp::os::Searchable& config);

    /**
     * Close the DeviceDriver.
     * @return true/false on success/failure.
     */
    virtual bool close();

// ------------------------------- Protected -------------------------------------

protected:

    /**
     * Check if index is within range (referred to driver vector size).
     * @param idx index to check.
     * @return true/false on success/failure.
     */
    bool indexWithinRange(const int& idx);

    /**
     * Check if number of joints is within range.
     * @param n_joint index to check.
     * @return true/false on success/failure.
     */
    bool batchWithinRange(const int& n_joint);

    /**
     * Convert from radians to degrees.
     * @param rad radians
     * @return degrees
     */
    static double toDeg(double rad);

    /**
     * Convert from degrees to radians.
     * @param deg degrees
     * @return radians
     */
    static double toRad(double deg);

// ------------------------------- Private -------------------------------------

private:

    AMOR_HANDLE handle;
    yarp::dev::PolyDriver cartesianControllerDevice;
    bool usingCartesianController;
    int controlMode;
};

}  // namespace roboticslab

#endif  // __AMOR_CONTROLBOARD_HPP__
