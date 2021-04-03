// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __FAKE_CONTROLBOARD_HPP__
#define __FAKE_CONTROLBOARD_HPP__

#include <vector>

#include <yarp/os/PeriodicThread.h>
#include <yarp/os/Semaphore.h>

#include <yarp/dev/Drivers.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/ControlBoardInterfaces.h>

#define DEFAULT_AXES 5

#define DEFAULT_EXTRA_ROBOT "none"
#define DEFAULT_EXTERN_OBJ "none"
#define DEFAULT_GEN_ENC_RAW_EXPOSED 0.0174532925199433  // Ratio, 0.0174532925199433 is pi/180 (raw/exp)<->(rad/deg)
#define DEFAULT_GEN_INIT_POS 0  // Exposed.
#define DEFAULT_GEN_JOINT_TOL 0.25  // Exposed.
#define DEFAULT_GEN_MAX_LIMIT 180.0  // Exposed.
#define DEFAULT_GEN_MIN_LIMIT -180.0  // Exposed.
#define DEFAULT_GEN_REF_SPEED 7.5  // Exposed.
#define DEFAULT_GEN_VEL_RAW_EXPOSED 0.0174532925199433  // Ratio, 0.0174532925199433 is pi/180 (raw/exp)<->(rad/deg)
#define DEFAULT_JMC_MS 20  // [ms]
#define DEFAULT_MODE_POS_VEL 0  // 0=Position, 1=Velocity.

namespace roboticslab
{

/**
 * @ingroup YarpPlugins
 * @defgroup EmulatedControlboard
 * @brief Contains roboticslab::EmulatedControlboard.
 */

/**
 * @ingroup EmulatedControlboard
 * @brief Implements several motor interfaces.
 */
class EmulatedControlboard : public yarp::dev::DeviceDriver,
                             public yarp::dev::IControlLimits,
                             public yarp::dev::IControlMode,
                             public yarp::dev::IEncodersTimed,
                             public yarp::dev::IPositionControl,
                             public yarp::dev::IPositionDirect,
                             public yarp::dev::IVelocityControl,
                             public yarp::os::PeriodicThread
{
public:

    // Set the thread period in the class constructor
    EmulatedControlboard() : PeriodicThread(DEFAULT_JMC_MS * 0.001) {}  // In seconds

// ------- IPositionControl declarations. Implementation in IPositionControlImpl.cpp -------

    /**
     * Get the number of controlled axes. This command asks the number of controlled
     * axes for the current physical interface.
     * @param ax pointer to storage
     * @return true/false.
     */
    bool getAxes(int *ax) override;

    /**
     * Set new reference point for a single axis.
     * @param j joint number
     * @param ref specifies the new ref point
     * @return true/false on success/failure
     */
    bool positionMove(int j, double ref) override;

    /**
     * Set new reference point for all axes.
     * @param refs array, new reference points.
     * @return true/false on success/failure
     */
    bool positionMove(const double *refs) override;

    /**
     * Set relative position. The command is relative to the
     * current position of the axis.
     * @param j joint axis number
     * @param delta relative command
     * @return true/false on success/failure
     */
    bool relativeMove(int j, double delta) override;

    /**
     * Set relative position, all joints.
     * @param deltas pointer to the relative commands
     * @return true/false on success/failure
     */
    bool relativeMove(const double *deltas) override;

    /**
     * Check if the current trajectory is terminated. Non blocking.
     * @return true if the trajectory is terminated, false otherwise
     */
    bool checkMotionDone(int j, bool *flag) override;

    /**
     * Check if the current trajectory is terminated. Non blocking.
     * @return true if the trajectory is terminated, false otherwise
     */
    bool checkMotionDone(bool *flag) override;

    /**
     * Set reference speed for a joint, this is the speed used during the
     * interpolation of the trajectory.
     * @param j joint number
     * @param sp speed value
     * @return true/false upon success/failure
     */
    bool setRefSpeed(int j, double sp) override;

    /**
     * Set reference speed on all joints. These values are used during the
     * interpolation of the trajectory.
     * @param spds pointer to the array of speed values.
     * @return true/false upon success/failure
     */
    bool setRefSpeeds(const double *spds) override;

    /**
     * Set reference acceleration for a joint. This value is used during the
     * trajectory generation.
     * @param j joint number
     * @param acc acceleration value
     * @return true/false upon success/failure
     */
    bool setRefAcceleration(int j, double acc) override;

    /**
     * Set reference acceleration on all joints. This is the valure that is
     * used during the generation of the trajectory.
     * @param accs pointer to the array of acceleration values
     * @return true/false upon success/failure
     */
    bool setRefAccelerations(const double *accs) override;

    /**
     * Get reference speed for a joint. Returns the speed used to
     * generate the trajectory profile.
     * @param j joint number
     * @param ref pointer to storage for the return value
     * @return true/false on success or failure
     */
    bool getRefSpeed(int j, double *ref) override;

    /**
     * Get reference speed of all joints. These are the  values used during the
     * interpolation of the trajectory.
     * @param spds pointer to the array that will store the speed values.
     */
    bool getRefSpeeds(double *spds) override;

    /**
     * Get reference acceleration for a joint. Returns the acceleration used to
     * generate the trajectory profile.
     * @param j joint number
     * @param acc pointer to storage for the return value
     * @return true/false on success/failure
     */
    bool getRefAcceleration(int j, double *acc) override;

    /**
     * Get reference acceleration of all joints. These are the values used during the
     * interpolation of the trajectory.
     * @param accs pointer to the array that will store the acceleration values.
     * @return true/false on success or failure
     */
    bool getRefAccelerations(double *accs) override;

    /**
     * Stop motion, single joint
     * @param j joint number
     * @return true/false on success/failure
     */
    bool stop(int j) override;

    /**
     * Stop motion, multiple joints
     * @return true/false on success/failure
     */
    bool stop() override;

    /**
     * Set new reference point for a subset of joints.
     * @param joints pointer to the array of joint numbers
     * @param refs   pointer to the array specifing the new reference points
     * @return true/false on success/failure
     */
    bool positionMove(const int n_joint, const int *joints, const double *refs) override;

    /**
     * Set relative position for a subset of joints.
     * @param joints pointer to the array of joint numbers
     * @param deltas pointer to the array of relative commands
     * @return true/false on success/failure
     */
    bool relativeMove(const int n_joint, const int *joints, const double *deltas) override;

    /**
     * Check if the current trajectory is terminated. Non blocking.
     * @param joints pointer to the array of joint numbers
     * @param flags  pointer to return value (logical "and" of all set of joints)
     * @return true/false if network communication went well.
     */
    bool checkMotionDone(const int n_joint, const int *joints, bool *flags) override;

    /**
     * Set reference speed on all joints. These values are used during the
     * interpolation of the trajectory.
     * @param joints pointer to the array of joint numbers
     * @param spds   pointer to the array with speed values.
     * @return true/false upon success/failure
     */
    bool setRefSpeeds(const int n_joint, const int *joints, const double *spds) override;

    /**
     * Set reference acceleration on all joints. This is the valure that is
     * used during the generation of the trajectory.
     * @param joints pointer to the array of joint numbers
     * @param accs   pointer to the array with acceleration values
     * @return true/false upon success/failure
     */
    bool setRefAccelerations(const int n_joint, const int *joints, const double *accs) override;

    /**
     * Get reference speed of all joints. These are the  values used during the
     * interpolation of the trajectory.
     * @param joints pointer to the array of joint numbers
     * @param spds   pointer to the array that will store the speed values.
     * @return true/false upon success/failure
     */
    bool getRefSpeeds(const int n_joint, const int *joints, double *spds) override;

    /**
     * Get reference acceleration for a joint. Returns the acceleration used to
     * generate the trajectory profile.
     * @param joints pointer to the array of joint numbers
     * @param accs   pointer to the array that will store the acceleration values
     * @return true/false on success/failure
     */
    bool getRefAccelerations(const int n_joint, const int *joints, double *accs) override;

    /**
     * Stop motion for subset of joints
     * @param joints pointer to the array of joint numbers
     * @return true/false on success/failure
     */
    bool stop(const int n_joint, const int *joints) override;

    /**
     * Get the last position reference for the specified axis.
     *  This is the dual of PositionMove and shall return only values sent using
     *  IPositionControl interface.
     *  If other interfaces like IPositionDirect are implemented by the device, this call
     *  must ignore their values, i.e. this call must never return a reference sent using
     *  IPositionDirect::SetPosition
     * @param ref last reference sent using PositionMove functions
     * @return true/false on success/failure
     */
    bool getTargetPosition(const int joint, double *ref) override;

    /**
     * Get the last position reference for all axes.
     *  This is the dual of PositionMove and shall return only values sent using
     *  IPositionControl interface.
     *  If other interfaces like IPositionDirect are implemented by the device, this call
     *  must ignore their values, i.e. this call must never return a reference sent using
     *  IPositionDirect::SetPosition
     * @param ref last reference sent using PositionMove functions
     * @return true/false on success/failure
     */
    bool getTargetPositions(double *refs) override;

    /**
     * Get the last position reference for the specified group of axes.
     *  This is the dual of PositionMove and shall return only values sent using
     *  IPositionControl interface.
     *  If other interfaces like IPositionDirect are implemented by the device, this call
     *  must ignore their values, i.e. this call must never return a reference sent using
     *  IPositionDirect::SetPosition
     * @param ref last reference sent using PositionMove functions
     * @return true/false on success/failure
     */
    bool getTargetPositions(const int n_joint, const int *joints, double *refs) override;

// ------- IPositionDirect declarations. Implementation in IPositionDirectImpl.cpp -------

    /**
     * Set new position for a single axis.
     * @param j joint number
     * @param ref specifies the new ref point
     * @return true/false on success/failure
     */
    bool setPosition(int j, double ref) override;

    /**
     * Set new reference point for all axes.
     * @param n_joint how many joints this command is referring to
     * @param joints list of joints controlled. The size of this array is n_joints
     * @param refs array, new reference points, one value for each joint, the size is n_joints.
     *        The first value will be the new reference fot the joint joints[0].
     *          for example:
     *          n_joint  3
     *          joints   0  2  4
     *          refs    10 30 40
     * @return true/false on success/failure
     */
    bool setPositions(const int n_joint, const int *joints, const double *refs) override;

    /**
     * Set new position for a set of axis.
     * @param refs specifies the new reference points
     * @return true/false on success/failure
     */
    bool setPositions(const double *refs) override;

    /**
     * Get the last position reference for the specified axis.
     *  This is the dual of setPositionsRaw and shall return only values sent using
     *  IPositionDirect interface.
     *  If other interfaces like IPositionControl are implemented by the device, this call
     *  must ignore their values, i.e. this call must never return a reference sent using
     *  IPositionControl::PositionMove.
     * @param ref last reference sent using setPosition(s) functions
     * @return true/false on success/failure
     */
    bool getRefPosition(const int joint, double *ref) override;

    /**
     * Get the last position reference for all axes.
     *  This is the dual of setPositionsRaw and shall return only values sent using
     *  IPositionDirect interface.
     *  If other interfaces like IPositionControl are implemented by the device, this call
     *  must ignore their values, i.e. this call must never return a reference sent using
     *  IPositionControl::PositionMove.
     * @param ref array containing last reference sent using setPosition(s) functions
     * @return true/false on success/failure
     */
    bool getRefPositions(double *refs) override;

    /**
     * Get the last position reference for the specified group of axes.
     *  This is the dual of setPositionsRaw and shall return only values sent using
     *  IPositionDirect interface.
     *  If other interfaces like IPositionControl are implemented by the device, this call
     *  must ignore their values, i.e. this call must never return a reference sent using
     *  IPositionControl::PositionMove.
     * @param ref array containing last reference sent using setPosition(s) functions
     * @return true/false on success/failure
     */
    bool getRefPositions(const int n_joint, const int *joints, double *refs) override;

//  ---------- IEncodersTimed Declarations. Implementation in IEncoderImpl.cpp ----------

    /**
     * Reset encoder, single joint. Set the encoder value to zero
     * @param j encoder number
     * @return true/false
     */
    bool resetEncoder(int j) override;

    /**
     * Reset encoders. Set the encoders value to zero
     * @return true/false
     */
    bool resetEncoders() override;

    /**
     * Set the value of the encoder for a given joint.
     * @param j encoder number
     * @param val new value
     * @return true/false
     */
    bool setEncoder(int j, double val) override;

    /**
     * Set the value of all encoders.
     * @param vals pointer to the new values
     * @return true/false
     */
    bool setEncoders(const double *vals) override;

    /**
     * Read the value of an encoder.
     * @param j encoder number
     * @param v pointer to storage for the return value
     * @return true/false, upon success/failure (you knew it, uh?)
     */
    bool getEncoder(int j, double *v) override;

    /**
     * Read the position of all axes.
     * @param encs pointer to the array that will contain the output
     * @return true/false on success/failure
     */
    bool getEncoders(double *encs) override;

    /**
     * Read the instantaneous speed of an axis.
     * @param j axis number
     * @param sp pointer to storage for the output
     * @return true if successful, false ... otherwise.
     */
    bool getEncoderSpeed(int j, double *sp) override;

    /**
     * Read the instantaneous speed of all axes.
     * @param spds pointer to storage for the output values
     * @return guess what? (true/false on success or failure).
     */
    bool getEncoderSpeeds(double *spds) override;

    /**
     * Read the instantaneous acceleration of an axis.
     * @param j axis number
     * @param spds pointer to the array that will contain the output
     */
    bool getEncoderAcceleration(int j, double *spds) override;

    /**
     * Read the instantaneous acceleration of all axes.
     * @param accs pointer to the array that will contain the output
     * @return true if all goes well, false if anything bad happens.
     */
    bool getEncoderAccelerations(double *accs) override;

    /**
     * Read the instantaneous acceleration of all axes.
     * @param encs pointer to the array that will contain the output
     * @param time pointer to the array that will contain individual timestamps
     * @return true if all goes well, false if anything bad happens.
     */
    bool getEncodersTimed(double *encs, double *time) override;

    /**
     * Read the instantaneous acceleration of all axes.
     * @param j axis index
     * @param encs encoder value (pointer to)
     * @param time corresponding timestamp (pointer to)
     * @return true if all goes well, false if anything bad happens.
     */
    bool getEncoderTimed(int j, double *encs, double *time) override;

//  --------- IVelocityControl Declarations. Implementation in IVelocityControlImpl.cpp ---------

    /**
     * Start motion at a given speed, single joint.
     * @param j joint number
     * @param sp speed value
     * @return bool/false upone success/failure
     */
    bool velocityMove(int j, double sp) override;

    /**
     * Start motion at a given speed, multiple joints.
     * @param sp pointer to the array containing the new speed values
     * @return true/false upon success/failure
     */
    bool velocityMove(const double *sp) override;

    /**
     * Start motion at a given speed for a subset of joints.
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
    bool velocityMove(const int n_joint, const int *joints, const double *spds) override;

    /**
     * Get the last reference speed set by velocityMove for single joint.
     * @param j joint number
     * @param vel returns the requested reference.
     * @return true/false on success/failure
     */
    bool getRefVelocity(const int joint, double *vel) override;

    /**
     * Get the last reference speed set by velocityMove for all joints.
     * @param vels pointer to the array containing the new speed values, one value for each joint
     * @return true/false on success/failure
     */
    bool getRefVelocities(double *vels) override;

    /**
     * Get the last reference speed set by velocityMove for a group of joints.
     * @param n_joint how many joints this command is referring to
     * @param joints of joints controlled. The size of this array is n_joints
     * @param vels pointer to the array containing the requested values, one value for each joint.
     *  The size of the array is n_joints.
     * @return true/false on success/failure
     */
    bool getRefVelocities(const int n_joint, const int *joints, double *vels) override;

//  --------- IControlLimits Declarations. Implementation in IControlLimitsImpl.cpp ---------

    /**
     * Set the software limits for a particular axis, the behavior of the
     * control card when these limits are exceeded, depends on the implementation.
     * @param axis joint number (why am I telling you this)
     * @param min the value of the lower limit
     * @param max the value of the upper limit
     * @return true or false on success or failure
     */
    bool setLimits(int axis, double min, double max) override;

    /**
     * Get the software limits for a particular axis.
     * @param axis joint number (again... why am I telling you this)
     * @param pointer to store the value of the lower limit
     * @param pointer to store the value of the upper limit
     * @return true if everything goes fine, false otherwise.
     */
    bool getLimits(int axis, double *min, double *max) override;

    /**
     * Set the software speed limits for a particular axis, the behavior of the
     * control card when these limits are exceeded, depends on the implementation.
     * @param axis joint number
     * @param min the value of the lower limit
     * @param max the value of the upper limit
     * @return true or false on success or failure
     */
    bool setVelLimits(int axis, double min, double max) override;

    /**
     * Get the software speed limits for a particular axis.
     * @param axis joint number
     * @param min pointer to store the value of the lower limit
     * @param max pointer to store the value of the upper limit
     * @return true if everything goes fine, false otherwise.
     */
    bool getVelLimits(int axis, double *min, double *max) override;

//  --------- IControlMode Declarations. Implementation in IControlModeImpl.cpp ---------

    /**
     * Get the current control mode.
     * @param j joint number
     * @param mode a vocab of the current control mode for joint j.
     * @return true/false success failure.
     */
    bool getControlMode(int j, int *mode) override;

    /**
     * Get the current control mode (multiple joints).
     * @param modes a vector containing vocabs for the current control modes of the joints.
     * @return true/false success failure.
     */
    bool getControlModes(int *modes) override;

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
    bool getControlModes(const int n_joint, const int *joints, int *modes) override;

    /**
     * Set the current control mode.
     * @param j joint number
     * @param mode a vocab of the desired control mode for joint j.
     * @return true if the new controlMode was successfully set, false if the message was not received or
     *         the joint was unable to switch to the desired controlMode
     *         (e.g. the joint is on a fault condition or the desired mode is not implemented).
     */
    bool setControlMode(const int j, const int mode) override;

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
    bool setControlModes(const int n_joint, const int *joints, int *modes) override;

    /**
     * Set the current control mode (multiple joints).
     * @param modes a vector containing vocabs for the desired control modes of the joints.
     * @return true if the new controlMode was successfully set, false if the message was not received or
     *         the joint was unable to switch to the desired controlMode
     *         (e.g. the joint is on a fault condition or the desired mode is not implemented).
     */
    bool setControlModes(int *modes) override;

// -------- DeviceDriver declarations. Implementation in DeviceDriverImpl.cpp --------

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
    bool open(yarp::os::Searchable& config) override;

    /**
     * Close the DeviceDriver.
     * @return true/false on success/failure.
     */
    bool close() override;

// -------- PeriodicThread declarations. Implementation in PeriodicThreadImpl.cpp --------

    /**
     * Initialization method. The thread executes this function
     * when it starts and before "run". This is a good place to
     * perform initialization tasks that need to be done by the
     * thread itself (device drivers initialization, memory
     * allocation etc). If the function returns false the thread
     * quits and never calls "run". The return value of threadInit()
     * is notified to the class and passed as a parameter
     * to afterStart(). Note that afterStart() is called by the
     * same thread that is executing the "start" method.
     */
    bool threadInit() override;

    /**
     * Loop function. This is the thread itself.
     */
    void run() override;

// ----- Shared Area Funcion declarations. Implementation in SharedArea.cpp -----

    void setEncRaw(const int index, const double position);
    void setEncsRaw(const std::vector<double> & positions);

    double getEncRaw(const int index);
    std::vector<double> getEncsRaw();

    double getEncExposed(const int index);
    std::vector<double> getEncsExposed();

// ------------------------------- Private -------------------------------------

private:

    enum jmc_state { NOT_CONTROLLING, POSITION_MOVE, RELATIVE_MOVE, VELOCITY_MOVE };
    enum jmc_mode { POSITION_MODE, VELOCITY_MODE, POSITION_DIRECT_MODE };

    bool setPositionMode(int j);
    bool setVelocityMode(int j);
    bool setTorqueMode(int j);
    bool setPositionDirectMode(int j);

    // General Joint Motion Controller parameters //
    unsigned int axes;
    double jmcMs;
    jmc_mode controlMode;
    double lastTime;

    yarp::os::Semaphore encRawMutex;  // SharedArea

    std::vector<jmc_state> jointStatus;

    std::vector<double> encRaw;
    std::vector<double> encRawExposed;  // For conversion.
    std::vector<double> initPos;  // Exposed.
    std::vector<double> jointTol;  // Exposed.
    std::vector<double> maxLimit;  // Exposed.
    std::vector<double> minLimit;  // Exposed.
    std::vector<double> refAcc;  // Exposed.
    std::vector<double> refSpeed;  // Exposed.
    std::vector<double> targetExposed;  // Exposed.
    std::vector<double> velRawExposed;  // For conversion.
    std::vector<double> velRaw;
};

} // namespace roboticslab

#endif // __FAKE_CONTROLBOARD_HPP__
