// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __TEXTILES_HAND__
#define __TEXTILES_HAND__

#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <yarp/dev/IControlLimits2.h>

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
#include "ColorDebug.h"
#include "ICanBusSharer.h"


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
*
*/
// Note: IEncodersTimedRaw inherits from IEncodersRaw
// Note: IControlLimits2Raw inherits from IControlLimitsRaw
class TextilesHand : public yarp::dev::DeviceDriver, public yarp::dev::IControlLimits2Raw, public yarp::dev::IControlMode2Raw, public yarp::dev::IEncodersTimedRaw,
    public yarp::dev::IPositionControl2Raw, public yarp::dev::IPositionDirectRaw, public yarp::dev::IVelocityControl2Raw, public yarp::dev::ITorqueControlRaw,
    public ICanBusSharer, public yarp::dev::IInteractionModeRaw
{

public:

    TextilesHand()
    {
        canDevicePtr = 0;
    }

    //  --------- DeviceDriver Declarations. Implementation in TextilesHand.cpp ---------
    virtual bool open(yarp::os::Searchable& config);
    virtual bool close();

    //  --------- ICanBusSharer Declarations. Implementation in TextilesHand.cpp ---------
    virtual bool setCanBusPtr(yarp::dev::ICanBus *canDevicePtr);
    virtual bool setIEncodersTimedRawExternal(IEncodersTimedRaw * iEncodersTimedRaw)
    {
        return true;
    }
    virtual bool interpretMessage(yarp::dev::CanMessage * message);
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

    //  --------- IControlLimitsRaw Declarations. Implementation in IControlLimitsRawImpl.cpp ---------
    virtual bool setLimitsRaw(int axis, double min, double max);
    virtual bool getLimitsRaw(int axis, double *min, double *max);
    virtual bool setVelLimitsRaw(int axis, double min, double max);
    virtual bool getVelLimitsRaw(int axis, double *min, double *max);

    //  --------- IControlModeRaw Declarations. Implementation in IControlMode2RawImpl.cpp ---------
    virtual bool getControlModeRaw(int j, int *mode);
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
    virtual bool setPositionsRaw(const int n_joint, const int *joints, const double *refs);
#if YARP_VERSION_MAJOR != 3
    virtual bool setPositionsRaw(const int n_joint, const int *joints, double *refs)
    {
        return setPositionsRaw(n_joint, joints, const_cast<const double *>(refs));
    }
#endif // YARP_VERSION_MAJOR != 3
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
#if YARP_VERSION_MAJOR != 3
    virtual bool getBemfParamRaw(int j, double *bemf);
    virtual bool setBemfParamRaw(int j, double bemf);
#endif // YARP_VERSION_MAJOR != 3

    //  --------- IVelocityControlRaw Declarations. Implementation in IVelocityControl2RawImpl.cpp ---------
    virtual bool velocityMoveRaw(int j, double sp);
    virtual bool velocityMoveRaw(const double *sp);

    //  --------- IVelocityControl2Raw Declarations. Implementation in IVelocityControl2RawImpl.cpp ---------
    virtual bool velocityMoveRaw(const int n_joint, const int *joints, const double *spds);
    virtual bool getRefVelocityRaw(const int joint, double *vel);
    virtual bool getRefVelocitiesRaw(double *vels);
    virtual bool getRefVelocitiesRaw(const int n_joint, const int *joints, double *vels);
    // -- (just defined in IInteractionModeRaw) - virtual bool setRefAccelerationsRaw(const int n_joint, const int *joints, const double *accs);
    // -- (just defined in IInteractionModeRaw) - virtual bool getRefAccelerationsRaw(const int n_joint, const int *joints, double *accs);
    // -- (just defined in IInteractionModeRaw) - virtual bool stopRaw(const int n_joint, const int *joints);

    // ------- IInteractionModeRaw declarations. Implementation in IInteractionModeRawImpl.cpp -------

    virtual bool getInteractionModeRaw(int axis, yarp::dev::InteractionModeEnum* mode);
    virtual bool getInteractionModesRaw(int n_joints, int *joints, yarp::dev::InteractionModeEnum* modes);
    virtual bool getInteractionModesRaw(yarp::dev::InteractionModeEnum* modes);
    virtual bool setInteractionModeRaw(int axis, yarp::dev::InteractionModeEnum mode);
    virtual bool setInteractionModesRaw(int n_joints, int *joints, yarp::dev::InteractionModeEnum* modes);
    virtual bool setInteractionModesRaw(yarp::dev::InteractionModeEnum* modes);

protected:

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


    /** pt-related **/
    int ptPointCounter;
    yarp::os::Semaphore ptBuffer;
    bool ptMovementDone;

    bool targetReached;

    int canId;

    yarp::dev::ICanBus *canDevicePtr;

    double max, min, refAcceleration, refSpeed, tr, targetPosition;

    double lastUsage;

    double encoder;
    uint32_t encoderTimestamp;
    yarp::os::Semaphore encoderReady;

    /** A helper function to display CAN messages. */
    std::string msgToStr(yarp::dev::CanMessage * message);
    std::string msgToStr(uint32_t cob, uint16_t len, uint8_t * msgData);

    int16_t ptModeMs;  //-- [ms]

    //-- Set the interaction mode of the robot for a set of joints, values can be stiff or compliant
    yarp::dev::InteractionModeEnum interactionMode;

    //-- Semaphores
    yarp::os::Semaphore interactionModeSemaphore;
};

}  // namespace roboticslab

#endif  // __TEXTILES_HAND__

