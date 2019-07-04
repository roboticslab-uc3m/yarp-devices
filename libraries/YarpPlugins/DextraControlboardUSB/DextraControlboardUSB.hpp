// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __DEXTRA_CONTROLBOARD_USB__
#define __DEXTRA_CONTROLBOARD_USB__

#include <stdint.h>

#include <yarp/os/Semaphore.h>

#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>

#include "Synapse.hpp"

#define DEFAULT_PORT "/dev/ttyACM0" // also /dev/ttyUSB0

namespace roboticslab
{

/**
 * @ingroup YarpPlugins
 * \defgroup DextraControlboardUSB
 * @brief Contains roboticslab::DextraControlboardUSB.
 */

/**
* @ingroup DextraControlboardUSB
* @brief Implementation for the custom UC3M Dextra Hand controlboard interfaces.
*
*/
class DextraControlboardUSB : public yarp::dev::DeviceDriver,
                              public yarp::dev::IControlLimits,
                              public yarp::dev::IControlMode,
                              public yarp::dev::IEncodersTimed,
                              public yarp::dev::IInteractionMode,
                              public yarp::dev::IPositionControl,
                              public yarp::dev::IPositionDirect,
                              public yarp::dev::ITorqueControl,
                              public yarp::dev::IVelocityControl
{

public:

    DextraControlboardUSB()
    {
    }

    //  --------- DeviceDriver Declarations. Implementation in DeviceDriverImpl.cpp ---------
    virtual bool open(yarp::os::Searchable& config);
    virtual bool close();

    //  --------- IControlLimits Declarations. Implementation in IControlLimitsImpl.cpp ---------
    virtual bool setLimits(int axis, double min, double max);
    virtual bool getLimits(int axis, double *min, double *max);
    virtual bool setVelLimits(int axis, double min, double max);
    virtual bool getVelLimits(int axis, double *min, double *max);

    //  --------- IControlMode Declarations. Implementation in IControlModeImpl.cpp ---------
    virtual bool getControlMode(int j, int *mode);
    virtual bool getControlModes(int *modes);
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

    // ------- IPositionControl declarations. Implementation in IPositionControlImpl.cpp -------
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
    virtual bool setPositions(const int n_joint, const int *joints, const double *refs);
    virtual bool setPositions(const double *refs);

    // -------- ITorqueControl declarations. Implementation in ITorqueControlImpl.cpp --------
    virtual bool getRefTorques(double *t);
    virtual bool getRefTorque(int j, double *t);
    virtual bool setRefTorques(const double *t);
    virtual bool setRefTorque(int j, double t);
    virtual bool getTorque(int j, double *t);
    virtual bool getTorques(double *t);
    virtual bool getTorqueRange(int j, double *min, double *max);
    virtual bool getTorqueRanges(double *min, double *max);

    //  --------- IVelocityControl Declarations. Implementation in IVelocityControlImpl.cpp ---------
    virtual bool velocityMove(int j, double sp);
    virtual bool velocityMove(const double *sp);
    virtual bool velocityMove(const int n_joint, const int *joints, const double *spds);
    virtual bool getRefVelocity(const int joint, double *vel);
    virtual bool getRefVelocities(double *vels);
    virtual bool getRefVelocities(const int n_joint, const int *joints, double *vels);
    // -- (just defined in IInteractionMode) - virtual bool setRefAccelerations(const int n_joint, const int *joints, const double *accs);
    // -- (just defined in IInteractionMode) - virtual bool getRefAccelerations(const int n_joint, const int *joints, double *accs);
    // -- (just defined in IInteractionMode) - virtual bool stop(const int n_joint, const int *joints);

    // ------- IInteractionMode declarations. Implementation in IInteractionModeImpl.cpp -------

    virtual bool getInteractionMode(int axis, yarp::dev::InteractionModeEnum* mode);
    virtual bool getInteractionModes(int n_joints, int *joints, yarp::dev::InteractionModeEnum* modes);
    virtual bool getInteractionModes(yarp::dev::InteractionModeEnum* modes);
    virtual bool setInteractionMode(int axis, yarp::dev::InteractionModeEnum mode);
    virtual bool setInteractionModes(int n_joints, int *joints, yarp::dev::InteractionModeEnum* modes);
    virtual bool setInteractionModes(yarp::dev::InteractionModeEnum* modes);

protected:

    //  --------- Implementation in DextraControlboardUSB.cpp ---------
    // takes the string name of the serial port (e.g. "/dev/tty.usbserial","COM1")
    // and a baud rate (bps) and connects to that port at that speed and 8N1.
    // opens the port in fully raw mode so you can send binary data.
    // returns valid fd, or -1 on error
    int serialport_init(const char* serialport, int baud);

    void serialport_close(int fd);

    int serialport_writebyte(int fd, uint8_t b);

    int serialport_write(int fd, const char* str);

    int serialport_read_until(int fd, char* buf, char until);

    Synapse synapse;

    yarp::dev::PolyDriver serialDevice;

    int fd;  // File descriptor for serial communications

    bool targetReached;

    double max, min, refAcceleration, refSpeed, tr, targetPosition;

    double lastUsage;

    double encoder;
    uint32_t encoderTimestamp;
    yarp::os::Semaphore encoderReady;

    //-- Set the interaction mode of the robot for a set of joints, values can be stiff or compliant
    yarp::dev::InteractionModeEnum interactionMode;

    //-- Semaphores
    yarp::os::Semaphore interactionModeSemaphore;
};

}  // namespace roboticslab

#endif  // __DEXTRA_CONTROLBOARD_USB__
