// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __CAN_BUS_CONTROLBOARD__
#define __CAN_BUS_CONTROLBOARD__

#include <yarp/os/all.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/Drivers.h>
#include <yarp/dev/PolyDriver.h>

#include <stdlib.h>  //-- Just for ::exit()
#include <fcntl.h>  //-- Just for O_RDWR
#include <vector>
#include <map>
#include <list>
#include <sstream>

//#define CD_HIDE_DEBUG  //-- Can be globally managed from father CMake.
//#define CD_HIDE_SUCCESS  //-- Can be globally managed from father CMake.
//#define CD_HIDE_INFO  //-- Can be managed from father CMake.
//#define CD_HIDE_WARNING  //-- Can be managed from father CMake.
//#define CD_HIDE_ERROR  //-- Can be managed from father CMake.
#include "ColorDebug.hpp"

#include "ICanBusSharer.h"

#define DEFAULT_MODE "position"

#define DEFAULT_PT_MODE_MS 50  //-- Don't move more than 1 degree in 50 ms.


namespace teo
{

/**
 *
 * @ingroup BodyYarp
 * \defgroup CanBusControlboard
 * @brief Contains teo::CanBusControlboard.
 */

 /**
 * @ingroup CanBusControlboard
 * @brief Implements IControlLimits, IControlMode, IEncodersTimed, IPositionControl, IPositionDirect,
 * ITorqueControl, IVelocityControl interface yarp::dev class member functions, linking to teo::TechnosoftIpos,
 * teo::LacqueyFetch and/or teo::FakeJoint raw implementations.
 *
 */
// Note: IEncodersTimed inherits from IEncoders
class CanBusControlboard : public yarp::dev::DeviceDriver, public yarp::dev::IControlLimits, public yarp::dev::IControlMode, public yarp::dev::IEncodersTimed,
        public yarp::dev::IPositionControl, public yarp::dev::IPositionDirect, public yarp::dev::ITorqueControl, public yarp::dev::IVelocityControl,
        public yarp::os::Thread {

    // ------------------------------- Public -------------------------------------

    public:

        //  --------- IControlLimits Declarations. Implementation in IControlLimitsImpl.cpp ---------

        /**
         * Set the software limits for a particular axis, the behavior of the
         * control card when these limits are exceeded, depends on the implementation.
         * @param axis joint number (why am I telling you this)
         * @param min the value of the lower limit
         * @param max the value of the upper limit
         * @return true or false on success or failure
         */
        virtual bool setLimits(int axis, double min, double max);

        /**
         * Get the software limits for a particular axis.
         * @param axis joint number (again... why am I telling you this)
         * @param pointer to store the value of the lower limit
         * @param pointer to store the value of the upper limit
         * @return true if everything goes fine, false otherwise.
         */
        virtual bool getLimits(int axis, double *min, double *max);

        //  --------- IControlMode Declarations. Implementation in IControlModeImpl.cpp ---------

        /**
        * Set position mode, single axis.
        * @param j: joint number
        * @return: true/false success failure.
        */
        virtual bool setPositionMode(int j);

        /**
        * Set velocity mode, single axis.
        * @param j: joint number
        * @return: true/false success failure.
        */
        virtual bool setVelocityMode(int j);

        /**
        * Set torque mode, single axis.
        * @param j: joint number
        * @return: true/false success failure.
        */
        virtual bool setTorqueMode(int j);

        /**
        * Set impedance position mode, single axis.
        * @param j: joint number
        * @return: true/false success failure.
        */
        virtual bool setImpedancePositionMode(int j);

        /**
        * Set impedance velocity mode, single axis.
        * @param j: joint number
        * @return: true/false success failure.
        */
        virtual bool setImpedanceVelocityMode(int j);

        /**
        * Set open loop mode, single axis.
        * @param j: joint number
        * @return: true/false success failure.
        */
        virtual bool setOpenLoopMode(int j);

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

        //  ---------- IEncoders Declarations. Implementation in IEncodersImpl.cpp ----------

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
         * Read the istantaneous speed of an axis.
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

        //  ---------- IEncodersTimed Declarations. Implementation in IEncodersTimedImpl.cpp ----------

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

        // ------- IPositionControl declarations. Implementation in IPositionControlImpl.cpp -------

        /**
         * Get the number of controlled axes. This command asks the number of controlled
         * axes for the current physical interface.
         * @param ax pointer to storage
         * @return true/false.
         */
        virtual bool getAxes(int *ax);

        /** Set position mode. This command
         * is required by control boards implementing different
         * control methods (e.g. velocity/torque), in some cases
         * it can be left empty.
         * return true/false on success/failure
         */
        virtual bool setPositionMode();

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

        // ------- IPositionDirect declarations. Implementation in IPositionDirectImpl.cpp -------

        /**
         * Set position direct mode. This command
         * is required to switch control boards to low-level position
         * control method.
         * @return true/false on success failure
         */
        virtual bool setPositionDirectMode();


        /** Set new position for a single axis.
         * @param j joint number
         * @param ref specifies the new ref point
         * @return true/false on success/failure
         */
        virtual bool setPosition(int j, double ref);

        /** Set new reference point for all axes.
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
        virtual bool setPositions(const int n_joint, const int *joints, double *refs);

        /** Set new position for a set of axis.
         * @param refs specifies the new reference points
         * @return true/false on success/failure
         */
        virtual bool setPositions(const double *refs);

        // -------- ITorqueControl declarations. Implementation in ITorqueControlImpl.cpp --------

        /**
         * Set torque control mode. This command
         * is required by control boards implementing different
         * control methods (e.g. velocity/torque), in some cases
         * it can be left empty.
         * @return true/false on success/failure
         */
        virtual bool setTorqueMode();

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

        /** Set the back-efm compensation gain for a given joint.
         * @param j joint number
         * @param bemf the returned bemf gain of joint j
         * @return true/false on success/failure
         */
        virtual bool getBemfParam(int j, double *bemf);

        /** Set the back-efm compensation gain for a given joint.
         * @param j joint number
         * @param bemf new value
         * @return true/false on success/failure
         */
        virtual bool setBemfParam(int j, double bemf);

         /** Set new pid value for a joint axis.
         * @param j joint number
         * @param pid new pid value
         * @return true/false on success/failure
         */
        virtual bool setTorquePid(int j, const yarp::dev::Pid &pid);

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

        /** Set new pid value on multiple axes.
         * @param pids pointer to a vector of pids
         * @return true/false upon success/failure
         */
        virtual bool setTorquePids(const yarp::dev::Pid *pids);

        /** Set the torque error limit for the controller on a specific joint
         * @param j joint number
         * @param limit limit value
         * @return true/false on success/failure
         */
        virtual bool setTorqueErrorLimit(int j, double limit);

        /** Get the torque error limit for the controller on all joints.
         * @param limits pointer to the vector with the new limits
         * @return true/false on success/failure
         */
        virtual bool setTorqueErrorLimits(const double *limits);

        /** Get the current torque error for a joint.
         * @param j joint number
         * @param err pointer to the storage for the return value
         * @return true/false on success failure
         */
        virtual bool getTorqueError(int j, double *err);

        /** Get the torque error of all joints.
         * @param errs pointer to the vector that will store the errors
         * @return true/false on success/failure
         */
        virtual bool getTorqueErrors(double *errs);

        /** Get the output of the controller (e.g. pwm value)
         * @param j joint number
         * @param out pointer to storage for return value
         * @return true/false on success/failure
         */
        virtual bool getTorquePidOutput(int j, double *out);

        /** Get the output of the controllers (e.g. pwm value)
         * @param outs pointer to the vector that will store the output values
         * @return true/false on success/failure
         */
        virtual bool getTorquePidOutputs(double *outs);

        /** Get current pid value for a specific joint.
         * @param j joint number
         * @param pid pointer to storage for the return value.
         * @return true/false on success/failure
         */
        virtual bool getTorquePid(int j, yarp::dev::Pid *pid);

        /** Get current pid value for a specific joint.
         * @param pids vector that will store the values of the pids.
         * @return true/false on success/failure
         */
        virtual bool getTorquePids(yarp::dev::Pid *pids);

        /** Get the torque error limit for the controller on a specific joint
         * @param j joint number
         * @param limit pointer to the result value
         * @return true/false on success/failure
         */
        virtual bool getTorqueErrorLimit(int j, double *limit);

        /** Get the torque error limit for all controllers
         * @param limits pointer to the array that will store the output
         * @return true/false on success/failure
         */
        virtual bool getTorqueErrorLimits(double *limits);

        /** Reset the controller of a given joint, usually sets the
         * current position of the joint as the reference value for the PID, and resets
         * the integrator.
         * @param j joint number
         * @return true/false on success/failure
         */
        virtual bool resetTorquePid(int j);

        /** Disable the pid computation for a joint
         * @param j joint number
         * @return true/false on success/failure
         */
        virtual bool disableTorquePid(int j);

        /** Enable the pid computation for a joint
         * @param j joint number
         * @return true/false on success/failure
         */
        virtual bool enableTorquePid(int j);

        /** Set offset value for a given pid
         * @param j joint number
         * @param v the new value
         * @return true/false on success/failure
         */
        virtual bool setTorqueOffset(int j, double v);

        //  --------- IVelocityControl Declarations. Implementation in IVelocityControlImpl.cpp ---------

        /**
         * Set velocity mode. This command
         * is required by control boards implementing different
         * control methods (e.g. velocity/torque), in some cases
         * it can be left empty.
         * @return true/false on success failure
         */
        virtual bool setVelocityMode();

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

        // -------- Thread declarations. Implementation in ThreadImpl.cpp --------

        /**
         * Main body of the new thread.
         * Override this method to do what you want.
         * After Thread::start is called, this
         * method will start running in a separate thread.
         * It is important that this method either keeps checking
         * Thread::isStopping to see if it should stop, or
         * you override the Thread::onStop method to interact
         * with it in some way to shut the new thread down.
         * There is no really reliable, portable way to stop
         * a thread cleanly unless that thread cooperates.
         */
        virtual void run();

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
        virtual bool open(yarp::os::Searchable& config);

        /**
         * Close the DeviceDriver.
         * @return true/false on success/failure.
         */
        virtual bool close();

    // ------------------------------- Protected -------------------------------------

    protected:

        /** A CAN device. */
        yarp::dev::PolyDriver canBusDevice;
        CanBusHico* iCanBus;

        /** A vector of CAN node objects. */
        std::vector< yarp::dev::PolyDriver* > nodes;
        std::vector< yarp::dev::IControlLimitsRaw* > iControlLimitsRaw;
        std::vector< yarp::dev::IControlModeRaw* > iControlModeRaw;
        std::vector< yarp::dev::IEncodersTimedRaw* > iEncodersTimedRaw;
        std::vector< yarp::dev::IPositionControlRaw* > iPositionControlRaw;
        std::vector< yarp::dev::IPositionDirectRaw* > iPositionDirectRaw;
        std::vector< yarp::dev::ITorqueControlRaw* > iTorqueControlRaw;
        std::vector< yarp::dev::IVelocityControlRaw* > iVelocityControlRaw;
        std::vector< ICanBusSharer* > iCanBusSharer;
        std::map< int, int > idxFromCanId;

        /** A helper function to display CAN messages. */
        std::string msgToStr(can_msg* message);

        /**
         * Check if index is within range (referred to driver vector size).
         * @param idx index to check.
         * @return true/false on success/failure.
         */
        bool indexWithinRange(const int& idx);
};

}  // namespace teo

#endif  //  __CAN_BUS_CONTROLBOARD__
