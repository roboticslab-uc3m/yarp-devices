// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __TEXTILES_HAND_HPP__
#define __TEXTILES_HAND_HPP__

#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/IControlMode.h>
#include <yarp/dev/IEncodersTimed.h>
#include <yarp/dev/IPositionControl.h>
#include <yarp/dev/IPositionDirect.h>
#include <yarp/dev/IVelocityControl.h>

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

namespace roboticslab
{

/**
 * @ingroup YarpPlugins
 * \defgroup TextilesHand
 * @brief Contains roboticslab::TextilesHand.
 */

/**
 * @ingroup TextilesHand
 * @brief Implementation for the custom UC3M Textiles Hand as a single CAN bus joint (controlboard raw interfaces).
 */
class TextilesHand : public yarp::dev::DeviceDriver,
                     public yarp::dev::IControlMode,
                     public yarp::dev::IEncodersTimed,
                     public yarp::dev::IPositionControl,
                     public yarp::dev::IPositionDirect,
                     public yarp::dev::IVelocityControl
{
public:

    TextilesHand()
        : lastTarget(0.0)
    { }

    //  --------- DeviceDriver Declarations. Implementation in DeviceDriverImpl.cpp ---------

    virtual bool open(yarp::os::Searchable & config) override;
    virtual bool close() override;

    //  --------- IControlMode Declarations. Implementation in IControlModeImpl.cpp ---------

    virtual bool getControlMode(int j, int * mode) override;
    virtual bool getControlModes(int * modes) override;
    virtual bool getControlModes(int n_joint, const int * joints, int * modes) override;
    virtual bool setControlMode(int j, int mode) override;
    virtual bool setControlModes(int * modes) override;
    virtual bool setControlModes(int n_joint, const int * joints, int * modes) override;

    //  ---------- IEncoders Declarations. Implementation in IEncodersImpl.cpp ----------

    //virtual bool getAxes(int * ax) override;
    virtual bool resetEncoder(int j) override { return false; }
    virtual bool resetEncoders() override { return false; }
    virtual bool setEncoder(int j, double val) override { return false; }
    virtual bool setEncoders(const double * vals) override { return false; }
    virtual bool getEncoder(int j, double * v) override { return false; }
    virtual bool getEncoders(double * encs) override { return false; }
    virtual bool getEncoderSpeed(int j, double * sp) override { return false; }
    virtual bool getEncoderSpeeds(double * spds) override { return false; }
    virtual bool getEncoderAcceleration(int j, double * spds) override { return false; }
    virtual bool getEncoderAccelerations(double * accs) override { return false; }

    //  ---------- IEncodersTimed Declarations. Implementation in IEncodersImpl.cpp ----------

    virtual bool getEncoderTimed(int j, double * encs, double * time) override { return false; }
    virtual bool getEncodersTimed(double * encs, double * time) override { return false; }

    // ------- IPositionControl declarations. Implementation in IPositionControlImpl.cpp -------

    //virtual bool getAxes(int * ax) override;
    virtual bool positionMove(int j, double ref) override { return false; }
    virtual bool positionMove(const double * refs) override { return false; }
    virtual bool positionMove(int n_joint, const int * joints, const double * refs) override { return false; }
    virtual bool relativeMove(int j, double delta) override { return false; }
    virtual bool relativeMove(const double * deltas) override { return false; }
    virtual bool relativeMove(int n_joint, const int * joints, const double * deltas) override { return false; }
    virtual bool checkMotionDone(int j, bool * flag) override { return false; }
    virtual bool checkMotionDone(bool * flag) override { return false; }
    virtual bool checkMotionDone(int n_joint, const int * joints, bool * flag) override { return false; }
    virtual bool setRefSpeed(int j, double sp) override { return false; }
    virtual bool setRefSpeeds(const double * spds) override { return false; }
    virtual bool setRefSpeeds(int n_joint, const int * joints, const double * spds) override { return false; }
    virtual bool setRefAcceleration(int j, double acc) override { return false; }
    virtual bool setRefAccelerations(const double * accs) override { return false; }
    virtual bool setRefAccelerations(int n_joint, const int * joints, const double * accs) override { return false; }
    virtual bool getRefSpeeds(int n_joint, const int * joints, double * spds) override { return false; }
    virtual bool getRefSpeed(int j, double * ref) override { return false; }
    virtual bool getRefSpeeds(double * spds) override { return false; }
    virtual bool getRefAcceleration(int j, double * acc) override { return false; }
    virtual bool getRefAccelerations(double * accs) override { return false; }
    virtual bool getRefAccelerations(int n_joint, const int * joints, double * accs) override { return false; }
    virtual bool stop(int j) override { return false; }
    virtual bool stop() override { return false; }
    virtual bool stop(int n_joint, const int *joints) override { return false; }
    virtual bool getTargetPosition(int joint, double * ref) override { return false; }
    virtual bool getTargetPositions(double * refs) override { return false; }
    virtual bool getTargetPositions(int n_joint, const int * joints, double * refs) override { return false; }

    // ------- IPositionDirect declarations. Implementation in IPositionDirectImpl.cpp -------

    virtual bool getAxes(int * ax) override;
    virtual bool setPosition(int j, double ref) override;
    virtual bool setPositions(const double * refs) override;
    virtual bool setPositions(int n_joint, const int * joints, const double * refs) override;
    virtual bool getRefPosition(int joint, double * ref) override;
    virtual bool getRefPositions(double * refs) override;
    virtual bool getRefPositions(int n_joint, const int * joints, double * refs) override;

    //  --------- IVelocityControl Declarations. Implementation in IVelocityControlImpl.cpp ---------

    //virtual bool getAxes(int * ax) override;
    virtual bool velocityMove(int j, double spd) override { return false; }
    virtual bool velocityMove(const double * spds) override { return false; }
    virtual bool velocityMove(int n_joint, const int * joints, const double * spds) override { return false; }
    virtual bool getRefVelocity(int joint, double * vel) override { return false; }
    virtual bool getRefVelocities(double * vels) override { return false; }
    virtual bool getRefVelocities(int n_joint, const int * joints, double * vels) override { return false; }
    //virtual bool setRefAcceleration(int j, double acc) override;
    //virtual bool setRefAccelerations(const double * accs) override;
    //virtual bool setRefAccelerations(int n_joint, const int * joints, const double * accs) override;
    //virtual bool getRefAcceleration(int j, double * acc) override;
    //virtual bool getRefAccelerations(double * accs) override;
    //virtual bool getRefAccelerations(int n_joint, const int * joints, double * accs) override;
    //virtual bool stop(int j) override;
    //virtual bool stop() override;
    //virtual bool stop(int n_joint, const int *joints) override;

private:

    //  --------- Implementation in TextilesHand.cpp ---------
    // takes the string name of the serial port (e.g. "/dev/tty.usbserial","COM1")
    // and a baud rate (bps) and connects to that port at that speed and 8N1.
    // opens the port in fully raw mode so you can send binary data.
    // returns valid fd, or -1 on error
    int serialport_init(const char * serialport, int baud);
    int serialport_writebyte(int fd, uint8_t b);
    int serialport_write(int fd, const char * str);
    int serialport_read_until(int fd, char * buf, char until);

    int fd;
    double lastTarget;
};

} // namespace roboticslab

#endif // __TEXTILES_HAND_HPP__
