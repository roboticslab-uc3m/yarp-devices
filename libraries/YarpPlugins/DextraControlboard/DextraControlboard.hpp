// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __DEXTRA_CONTROLBOARD__
#define __DEXTRA_CONTROLBOARD__

#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <sstream>

#include <errno.h>    /* Error number definitions */
#include <fcntl.h>    /* File control definitions */
#include <stdio.h>
#include <stdint.h>   /* Standard types */
#include <stdlib.h>
#include <string.h>   /* String function definitions */
#include <unistd.h>   /* UNIX standard function definitions */
#include <termios.h>  /* POSIX terminal control definitions */
#include <sys/ioctl.h>
#include <getopt.h>

//#define CD_FULL_FILE  //-- Can be globally managed from father CMake. Good for debugging with polymorphism.
//#define CD_HIDE_DEBUG  //-- Can be globally managed from father CMake.
//#define CD_HIDE_SUCCESS  //-- Can be globally managed from father CMake.
//#define CD_HIDE_INFO  //-- Can be globally managed from father CMake.
//#define CD_HIDE_WARNING  //-- Can be globally managed from father CMake.
//#define CD_HIDE_ERROR  //-- Can be globally managed from father CMake.
#include "ColorDebug.hpp"
#include "ICanBusSharer.h"


namespace roboticslab
{

/**
 * @ingroup YarpPlugins
 * \defgroup DextraControlboard
 * @brief Contains roboticslab::DextraControlboard.
 */

/**
* @ingroup DextraControlboard
* @brief Implementation for the custom UC3M Dextra Hand controlboard interfaces.
*
*/
// Note: IEncodersTimed inherits from IEncoders
// Note: IControlLimits2 inherits from IControlLimits
class DextraControlboard : public yarp::dev::DeviceDriver, public yarp::dev::IControlLimits2, public yarp::dev::IControlMode2, public yarp::dev::IEncodersTimed,
    public yarp::dev::IPositionControl2, public yarp::dev::IPositionDirect, public yarp::dev::IVelocityControl2, public yarp::dev::ITorqueControl,
    public yarp::dev::IInteractionMode
{

public:

    DextraControlboard()
    {
        canDevicePtr = 0;
    }

    //  --------- DeviceDriver Declarations. Implementation in DextraControlboard.cpp ---------
    virtual bool open(yarp::os::Searchable& config);
    virtual bool close();

    //  --------- IControlLimits Declarations. Implementation in IControlLimitsImpl.cpp ---------
    virtual bool setLimits(int axis, double min, double max);
    virtual bool getLimits(int axis, double *min, double *max);
    virtual bool setVelLimits(int axis, double min, double max);
    virtual bool getVelLimits(int axis, double *min, double *max);

    //  --------- IControlMode Declarations. Implementation in IControlMode2Impl.cpp ---------
    virtual bool setPositionMode(int j);
    virtual bool setVelocityMode(int j);
    virtual bool setTorqueMode(int j);
    virtual bool setImpedancePositionMode(int j);
    virtual bool setImpedanceVelocityMode(int j);
    virtual bool setOpenLoopMode(int j);
    virtual bool getControlMode(int j, int *mode);
    virtual bool getControlModes(int *modes);

    //  --------- IControlMode2 Declarations. Implementation in IControlMode2Impl.cpp ---------
    virtual bool getControlModes(const int n_joint, const int *joints, int *modes);
    virtual bool setControlMode(const int j, const int mode);
    virtual bool setControlModes(const int n_joint, const int *joints, int *modes);
    virtual bool setControlModes(int *modes);

    //  ---------- IEncoders Declarations. Implementation in IEncodersImpl.cpp ----------
    virtual bool resetEncoder(int j);
    virtual bool resetEncoders();
    virtual bool setEncoder(int j, double val);
    virtual bool setEncoders(const double *vals);
    virtual bool getEncoder(int j, double *v);
    virtual bool getEncoders(double *encs);
    virtual bool getEncoderSpeed(int j, double *sp);
    virtual bool getEncoderSpeeds(double *spds);
    virtual bool getEncoderAcceleration(int j, double *spds);
    virtual bool getEncoderAccelerations(double *accs);

    //  ---------- IEncodersTimed Declarations. Implementation in IEncodersTimedImpl.cpp ----------
    virtual bool getEncodersTimed(double *encs, double *time);
    virtual bool getEncoderTimed(int j, double *encs, double *time);

    // ------- IPositionControl declarations. Implementation in IPositionControl2Impl.cpp -------
    virtual bool getAxes(int *ax);
    virtual bool positionMove(int j, double ref);
    virtual bool positionMove(const double *refs);
    virtual bool relativeMove(int j, double delta);
    virtual bool relativeMove(const double *deltas);
    virtual bool checkMotionDone(int j, bool *flag);
    virtual bool checkMotionDone(bool *flag);
    virtual bool setRefSpeed(int j, double sp);
    virtual bool setRefSpeeds(const double *spds);
    virtual bool setRefAcceleration(int j, double acc);
    virtual bool setRefAccelerations(const double *accs);
    virtual bool getRefSpeed(int j, double *ref);
    virtual bool getRefSpeeds(double *spds);
    virtual bool getRefAcceleration(int j, double *acc);
    virtual bool getRefAccelerations(double *accs);
    virtual bool stop(int j);
    virtual bool stop();

    // ------- IPositionControl2 declarations. Implementation in IPositionControl2Impl.cpp ---------

    virtual bool positionMove(const int n_joint, const int *joints, const double *refs);
    virtual bool relativeMove(const int n_joint, const int *joints, const double *deltas);
    virtual bool checkMotionDone(const int n_joint, const int *joints, bool *flags);
    virtual bool setRefSpeeds(const int n_joint, const int *joints, const double *spds);
    virtual bool setRefAccelerations(const int n_joint, const int *joints, const double *accs);
    virtual bool getRefSpeeds(const int n_joint, const int *joints, double *spds);
    virtual bool getRefAccelerations(const int n_joint, const int *joints, double *accs);
    virtual bool stop(const int n_joint, const int *joints);
    virtual bool getTargetPosition(const int joint, double *ref);
    virtual bool getTargetPositions(double *refs);
    virtual bool getTargetPositions(const int n_joint, const int *joints, double *refs);

    // ------- IPositionDirect declarations. Implementation in IPositionDirectImpl.cpp -------
    virtual bool setPosition(int j, double ref);
    virtual bool setPositions(const int n_joint, const int *joints, double *refs);
    virtual bool setPositions(const double *refs);

    // -------- ITorqueControl declarations. Implementation in ITorqueControlImpl.cpp --------
    virtual bool getRefTorques(double *t);
    virtual bool getRefTorque(int j, double *t);
    virtual bool setRefTorques(const double *t);
    virtual bool setRefTorque(int j, double t);
    virtual bool getBemfParam(int j, double *bemf);
    virtual bool setBemfParam(int j, double bemf);
    virtual bool setTorquePid(int j, const yarp::dev::Pid &pid);
    virtual bool getTorque(int j, double *t);
    virtual bool getTorques(double *t);
    virtual bool getTorqueRange(int j, double *min, double *max);
    virtual bool getTorqueRanges(double *min, double *max);
    virtual bool setTorquePids(const yarp::dev::Pid *pids);
    virtual bool setTorqueErrorLimit(int j, double limit);
    virtual bool setTorqueErrorLimits(const double *limits);
    virtual bool getTorqueError(int j, double *err);
    virtual bool getTorqueErrors(double *errs);
    virtual bool getTorquePidOutput(int j, double *out);
    virtual bool getTorquePidOutputs(double *outs);
    virtual bool getTorquePid(int j, yarp::dev::Pid *pid);
    virtual bool getTorquePids(yarp::dev::Pid *pids);
    virtual bool getTorqueErrorLimit(int j, double *limit);
    virtual bool getTorqueErrorLimits(double *limits);
    virtual bool resetTorquePid(int j);
    virtual bool disableTorquePid(int j);
    virtual bool enableTorquePid(int j);
    virtual bool setTorqueOffset(int j, double v);

    //  --------- IVelocityControl Declarations. Implementation in IVelocityControl2Impl.cpp ---------
    virtual bool velocityMove(int j, double sp);
    virtual bool velocityMove(const double *sp);

    //  --------- IVelocityControl2 Declarations. Implementation in IVelocityControl2Impl.cpp ---------
    virtual bool velocityMove(const int n_joint, const int *joints, const double *spds);
    virtual bool getRefVelocity(const int joint, double *vel);
    virtual bool getRefVelocities(double *vels);
    virtual bool getRefVelocities(const int n_joint, const int *joints, double *vels);
    // -- (just defined in IInteractionMode) - virtual bool setRefAccelerations(const int n_joint, const int *joints, const double *accs);
    // -- (just defined in IInteractionMode) - virtual bool getRefAccelerations(const int n_joint, const int *joints, double *accs);
    // -- (just defined in IInteractionMode) - virtual bool stop(const int n_joint, const int *joints);
    virtual bool setVelPid(int j, const yarp::dev::Pid &pid);
    virtual bool setVelPids(const yarp::dev::Pid *pids);
    virtual bool getVelPid(int j, yarp::dev::Pid *pid);
    virtual bool getVelPids(yarp::dev::Pid *pids);

    // ------- IInteractionMode declarations. Implementation in IInteractionModeImpl.cpp -------

    virtual bool getInteractionMode(int axis, yarp::dev::InteractionModeEnum* mode);
    virtual bool getInteractionModes(int n_joints, int *joints, yarp::dev::InteractionModeEnum* modes);
    virtual bool getInteractionModes(yarp::dev::InteractionModeEnum* modes);
    virtual bool setInteractionMode(int axis, yarp::dev::InteractionModeEnum mode);
    virtual bool setInteractionModes(int n_joints, int *joints, yarp::dev::InteractionModeEnum* modes);
    virtual bool setInteractionModes(yarp::dev::InteractionModeEnum* modes);

protected:

    //  --------- Implementation in DextraControlboard.cpp ---------
    // takes the string name of the serial port (e.g. "/dev/tty.usbserial","COM1")
    // and a baud rate (bps) and connects to that port at that speed and 8N1.
    // opens the port in fully raw mode so you can send binary data.
    // returns valid fd, or -1 on error
    int serialport_init(const char* serialport, int baud);

    int serialport_writebyte(int fd, uint8_t b);

    int serialport_write(int fd, const char* str);

    int serialport_read_until(int fd, char* buf, char until);

    int fd;  // File descriptor for serial communications


    /** pt-related **/
    int ptPointCounter;
    yarp::os::Semaphore ptBuffer;
    bool ptMovementDone;

    bool targetReached;

    int canId;

    ICanBusHico *canDevicePtr;

    double max, min, refAcceleration, refSpeed, tr, targetPosition;

    double lastUsage;

    double encoder;
    uint32_t encoderTimestamp;
    yarp::os::Semaphore encoderReady;

    /** A helper function to display CAN messages. */
    std::string msgToStr(can_msg* message);
    std::string msgToStr(uint32_t cob, uint16_t len, uint8_t * msgData);

    int16_t ptModeMs;  //-- [ms]

    //-- Set the interaction mode of the robot for a set of joints, values can be stiff or compliant
    yarp::dev::InteractionModeEnum interactionMode;

    //-- Semaphores
    yarp::os::Semaphore interactionModeSemaphore;
};

}  // namespace roboticslab

#endif  // __DEXTRA_CONTROLBOARD__

