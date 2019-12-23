// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __TEXTILES_HAND_HPP__
#define __TEXTILES_HAND_HPP__

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

#include "ColorDebug.h"
#include "ICanBusSharer.hpp"

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
                     public yarp::dev::IControlModeRaw,
                     public yarp::dev::IEncodersTimedRaw,
                     public yarp::dev::IPositionControlRaw,
                     public yarp::dev::IPositionDirectRaw,
                     public yarp::dev::IVelocityControlRaw,
                     public ICanBusSharer
{
public:

    TextilesHand()
    {}

    //  --------- DeviceDriver Declarations. Implementation in TextilesHand.cpp ---------
    virtual bool open(yarp::os::Searchable& config);
    virtual bool close();

    //  --------- ICanBusSharer Declarations. Implementation in TextilesHand.cpp ---------
    virtual unsigned int getId();
    virtual bool interpretMessage(const yarp::dev::CanMessage & message);
    virtual bool initialize();
    virtual bool finalize();
    virtual bool registerSender(CanSenderDelegate * sender);

    //  --------- IControlModeRaw Declarations. Implementation in IControlModeRawImpl.cpp ---------
    virtual bool getControlModeRaw(int j, int *mode);
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

    //  --------- IVelocityControlRaw Declarations. Implementation in IVelocityControlRawImpl.cpp ---------
    virtual bool velocityMoveRaw(int j, double sp);
    virtual bool velocityMoveRaw(const double *sp);
    virtual bool velocityMoveRaw(const int n_joint, const int *joints, const double *spds);
    virtual bool getRefVelocityRaw(const int joint, double *vel);
    virtual bool getRefVelocitiesRaw(double *vels);
    virtual bool getRefVelocitiesRaw(const int n_joint, const int *joints, double *vels);
    // -- (just defined in IInteractionModeRaw) - virtual bool setRefAccelerationsRaw(const int n_joint, const int *joints, const double *accs);
    // -- (just defined in IInteractionModeRaw) - virtual bool getRefAccelerationsRaw(const int n_joint, const int *joints, double *accs);
    // -- (just defined in IInteractionModeRaw) - virtual bool stopRaw(const int n_joint, const int *joints);

private:

    //  --------- Implementation in TextilesHand.cpp ---------
    // takes the string name of the serial port (e.g. "/dev/tty.usbserial","COM1")
    // and a baud rate (bps) and connects to that port at that speed and 8N1.
    // opens the port in fully raw mode so you can send binary data.
    // returns valid fd, or -1 on error
    int serialport_init(const char* serialport, int baud);

    int serialport_writebyte(int fd, uint8_t b);

    int serialport_write(int fd, const char* str);

    int serialport_read_until(int fd, char* buf, char until);

    int fd;  // File descriptor for serial communications

    bool targetReached;

    int canId;

    double max, min, refAcceleration, refSpeed, tr, targetPosition;

    double lastUsage;

    double encoder;
    uint32_t encoderTimestamp;
    yarp::os::Semaphore encoderReady;
};

} // namespace roboticslab

#endif // __TEXTILES_HAND_HPP__
