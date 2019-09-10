// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __CUI_ABSOLUTE__
#define __CUI_ABSOLUTE__

#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <yarp/dev/IControlLimits.h>
#include <yarp/dev/IRemoteVariables.h>

#include <sstream>
#include <math.h>

//#define CD_FULL_FILE  //-- Can be globally managed from father CMake. Good for debugging with polymorphism.
//#define CD_HIDE_DEBUG  //-- Can be globally managed from father CMake.
//#define CD_HIDE_SUCCESS  //-- Can be globally managed from father CMake.
//#define CD_HIDE_INFO  //-- Can be globally managed from father CMake.
//#define CD_HIDE_WARNING  //-- Can be globally managed from father CMake.
//#define CD_HIDE_ERROR  //-- Can be globally managed from father CMake.
#include "ColorDebug.h"
#include "ICanBusSharer.hpp"
#include "ICuiAbsolute.h"

namespace roboticslab
{

/**
 * @ingroup YarpPlugins
 * \defgroup CuiAbsolute
 * @brief Contains roboticslab::CuiAbsolute.
 */

/**
* @ingroup CuiAbsolute
* @brief Implementation for the Cui Absolute Encoder custom UC3M circuit as a single CAN bus joint (controlboard raw interfaces).
*
*/
class CuiAbsolute : public yarp::dev::DeviceDriver,
                    public yarp::dev::IControlLimitsRaw,
                    public yarp::dev::IControlModeRaw,
                    public yarp::dev::ICurrentControlRaw,
                    public yarp::dev::IEncodersTimedRaw,
                    public yarp::dev::IInteractionModeRaw,
                    public yarp::dev::IPositionControlRaw,
                    public yarp::dev::IPositionDirectRaw,
                    public yarp::dev::IRemoteVariablesRaw,
                    public yarp::dev::IVelocityControlRaw,
                    public yarp::dev::ITorqueControlRaw,
                    public ICanBusSharer,
                    public ICuiAbsolute
{

public:

    CuiAbsolute()
    {
        sender = 0;
        firstHasReached = false;
    }

    virtual bool HasFirstReached() {
        return firstHasReached;
    }

    //  --------- DeviceDriver Declarations. Implementation in CuiAbsolute.cpp ---------
    virtual bool open(yarp::os::Searchable& config);
    virtual bool close();

    //  --------- ICanBusSharer Declarations. Implementation in CuiAbsolute.cpp ---------
    virtual bool setIEncodersTimedRawExternal(IEncodersTimedRaw * iEncodersTimedRaw);
    virtual bool interpretMessage(const yarp::dev::CanMessage & message);
    /** "start". Figure 5.1 Drive’s status machine. States and transitions (p68, 84/263). */
    virtual bool initialize();
    virtual bool start();
    /** "ready to switch on", also acts as "shutdown" */
    virtual bool readyToSwitchOn();
    /** "switch on", also acts as "disable operation" */
    virtual bool switchOn();
    /** enable */
    virtual bool enable();
    /** recoverFromError */
    virtual bool recoverFromError();
    virtual bool registerSender(CanSenderDelegate * sender);

    //  --------- IControlLimitsRaw Declarations. Implementation in IControlLimitsRawImpl.cpp ---------
    virtual bool setLimitsRaw(int axis, double min, double max);
    virtual bool getLimitsRaw(int axis, double *min, double *max);
    virtual bool setVelLimitsRaw(int axis, double min, double max);
    virtual bool getVelLimitsRaw(int axis, double *min, double *max);

    //  --------- IControlModeRaw Declarations. Implementation in IControlModeRawImpl.cpp ---------
    virtual bool getControlModeRaw(int j, int *mode);
    virtual bool getControlModesRaw(int *modes);
    virtual bool getControlModesRaw(const int n_joint, const int *joints, int *modes);
    virtual bool setControlModeRaw(const int j, const int mode);
    virtual bool setControlModesRaw(const int n_joint, const int *joints, int *modes);
    virtual bool setControlModesRaw(int *modes);

    //  --------- ICurrentControlRaw Declarations. Implementation in ICurrentControlRawImpl.cpp ---------
    virtual bool getNumberOfMotorsRaw(int *number) { return false; }
    virtual bool getCurrentRaw(int m, double *curr) { return false; }
    virtual bool getCurrentsRaw(double *currs) { return false; }
    virtual bool getCurrentRangeRaw(int m, double *min, double *max) { return false; }
    virtual bool getCurrentRangesRaw(double *min, double *max) { return false; }
    virtual bool setRefCurrentsRaw(const double *currs) { return false; }
    virtual bool setRefCurrentRaw(int m, double curr) { return false; }
    virtual bool setRefCurrentsRaw(const int n_motor, const int *motors, const double *currs) { return false; }
    virtual bool getRefCurrentsRaw(double *currs) { return false; }
    virtual bool getRefCurrentRaw(int m, double *curr) { return false; }

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

    // -------- ITorqueControlRaw declarations. Implementation in ITorqueControlRawImpl.cpp --------
    virtual bool getRefTorquesRaw(double *t);
    virtual bool getRefTorqueRaw(int j, double *t);
    virtual bool setRefTorquesRaw(const double *t);
    virtual bool setRefTorqueRaw(int j, double t);
    virtual bool getTorqueRaw(int j, double *t);
    virtual bool getTorquesRaw(double *t);
    virtual bool getTorqueRangeRaw(int j, double *min, double *max);
    virtual bool getTorqueRangesRaw(double *min, double *max);

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

    // ------- IInteractionModeRaw declarations. Implementation in IInteractionModeRawImpl.cpp -------

    virtual bool getInteractionModeRaw(int axis, yarp::dev::InteractionModeEnum* mode);
    virtual bool getInteractionModesRaw(int n_joints, int *joints, yarp::dev::InteractionModeEnum* modes);
    virtual bool getInteractionModesRaw(yarp::dev::InteractionModeEnum* modes);
    virtual bool setInteractionModeRaw(int axis, yarp::dev::InteractionModeEnum mode);
    virtual bool setInteractionModesRaw(int n_joints, int *joints, yarp::dev::InteractionModeEnum* modes);
    virtual bool setInteractionModesRaw(yarp::dev::InteractionModeEnum* modes);

    // ------- IRemoteVariablesRaw declarations. Implementation in IRemoteVariablesRawImpl.cpp -------
    virtual bool getRemoteVariableRaw(std::string key, yarp::os::Bottle& val);
    virtual bool setRemoteVariableRaw(std::string key, const yarp::os::Bottle& val);
    virtual bool getRemoteVariablesListRaw(yarp::os::Bottle* listOfKeys);

    // -- Auxiliary functions: send data to PIC of Cui

    virtual bool startContinuousPublishing(uint8_t time);
    virtual bool startPullPublishing();
    virtual bool stopPublishingMessages();



protected:


    //  --------- Implementation in CuiAbsolute.cpp ---------
    /**
     * Write message to the CAN buffer.
     * @param cob Message's COB
     * @param len Data field length
     * @param msgData Data to send
     * @return true/false on success/failure.
     */
    bool send(uint32_t cob, uint16_t len, uint8_t * msgData);

    bool targetReached;
    int cuiTimeout;
    int canId;

    CanSenderDelegate * sender;

    double max, min, refAcceleration, refSpeed, tr, targetPosition;

    //-- Encoder stuff
    double encoder;
    double encoderTimestamp;
    yarp::os::Semaphore encoderReady;
    bool firstHasReached;

    //-- Set the interaction mode of the robot for a set of joints, values can be stiff or compliant
    yarp::dev::InteractionModeEnum interactionMode;

    //-- Semaphores
    yarp::os::Semaphore interactionModeSemaphore;
    //yarp::os::Semaphore targetPositionSemaphore;
    //yarp::os::Semaphore targetReachedReady;
    //yarp::os::Semaphore refSpeedSemaphore;
    //yarp::os::Semaphore refAccelSemaphore;
};

}  // namespace roboticslab

#endif  // __CUI_ABSOLUTE__

