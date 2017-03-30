// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __FAKE_JOINT__
#define __FAKE_JOINT__

#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <sstream>

//#define CD_FULL_FILE  //-- Can be globally managed from father CMake. Good for debugging with polymorphism.
//#define CD_HIDE_DEBUG  //-- Can be globally managed from father CMake.
//#define CD_HIDE_SUCCESS  //-- Can be globally managed from father CMake.
//#define CD_HIDE_INFO  //-- Can be globally managed from father CMake.
//#define CD_HIDE_WARNING  //-- Can be globally managed from father CMake.
//#define CD_HIDE_ERROR  //-- Can be globally managed from father CMake.
#include "ColorDebug.hpp"
#include "ICanBusSharer.h"


namespace teo
{

/**
 * @ingroup BodyYarp
 * \defgroup FakeJoint
 * @brief Contains teo::FakeJoint.
 */

/**
* @ingroup FakeJoint
* @brief Implementation for a fake joint (instant movement) as a single CAN bus joint (controlboard raw interfaces).
*
*/
// Note: IEncodersTimedRaw inherits from IEncodersRaw
// Note: IControlLimits2Raw inherits from IControlLimitsRaw
class FakeJoint : public yarp::dev::DeviceDriver, public yarp::dev::IControlLimits2Raw, public yarp::dev::IControlModeRaw, public yarp::dev::IEncodersTimedRaw,
    public yarp::dev::IPositionControl2Raw, public yarp::dev::IPositionDirectRaw, public yarp::dev::IVelocityControlRaw, public yarp::dev::ITorqueControlRaw,
    public ICanBusSharer
{

public:

    FakeJoint()
    {
        canDevicePtr = 0;
    }

    //  --------- DeviceDriver Declarations. Implementation in FakeJoint.cpp ---------
    virtual bool open(yarp::os::Searchable& config);
    virtual bool close();

    //  --------- ICanBusSharer Declarations. Implementation in FakeJoint.cpp ---------
    virtual bool setCanBusPtr(ICanBusHico *canDevicePtr);
    virtual bool setIEncodersTimedRawExternal(IEncodersTimedRaw * iEncodersTimedRaw)
    {
        return true;
    }
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

    //  --------- IControlLimitsRaw Declarations. Implementation in IControlLimitsRawImpl.cpp ---------
    virtual bool setLimitsRaw(int axis, double min, double max);
    virtual bool getLimitsRaw(int axis, double *min, double *max);
    virtual bool setVelLimitsRaw(int axis, double min, double max);
    virtual bool getVelLimitsRaw(int axis, double *min, double *max);

    //  --------- IControlModeRaw Declarations. Implementation in IControlModeRawImpl.cpp ---------
    virtual bool setPositionModeRaw(int j);
    virtual bool setVelocityModeRaw(int j);
    virtual bool setTorqueModeRaw(int j);
    virtual bool setImpedancePositionModeRaw(int j);
    virtual bool setImpedanceVelocityModeRaw(int j);
    virtual bool setOpenLoopModeRaw(int j);
    virtual bool getControlModeRaw(int j, int *mode);
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
    virtual bool getAxes(int *ax);
    virtual bool setPositionModeRaw();
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
    virtual bool setPositionDirectModeRaw()
    {
        CD_DEBUG("\n");
        return true;
    }
    virtual bool setPositionRaw(int j, double ref)
    {
        CD_DEBUG("\n");
        this->positionMoveRaw(0,ref);
        return true;
    }
    virtual bool setPositionsRaw(const int n_joint, const int *joints, double *refs)
    {
        CD_DEBUG("\n");
        this->positionMoveRaw(0,refs[0]);
        return true;
    }
    virtual bool setPositionsRaw(const double *refs)
    {
        CD_DEBUG("\n");
        return true;
    }

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
    virtual bool getRefTorqueRaw(int j, double *t)
    {
        CD_INFO("\n");
        return true;
    }
    virtual bool setRefTorquesRaw(const double *t)
    {
        CD_ERROR("\n");
        return false;
    }
    virtual bool setRefTorqueRaw(int j, double t)
    {
        CD_INFO("\n");
        return true;
    }
    virtual bool getBemfParamRaw(int j, double *bemf)
    {
        CD_INFO("\n");
        return true;
    }
    virtual bool setBemfParamRaw(int j, double bemf)
    {
        CD_INFO("\n");
        return true;
    }
    virtual bool setTorquePidRaw(int j, const yarp::dev::Pid &pid)
    {
        CD_INFO("\n");
        return true;
    }
    virtual bool getTorqueRaw(int j, double *t)
    {
        //CD_INFO("\n");  //-- Too verbose in controlboardwrapper2 stream.
        return true;
    }
    virtual bool getTorquesRaw(double *t)
    {
        CD_ERROR("\n");
        return false;
    }
    virtual bool getTorqueRangeRaw(int j, double *min, double *max)
    {
        CD_INFO("\n");
        return true;
    }
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
    virtual bool setTorqueErrorLimitRaw(int j, double limit)
    {
        CD_INFO("\n");
        return true;
    }
    virtual bool setTorqueErrorLimitsRaw(const double *limits)
    {
        CD_ERROR("\n");
        return false;
    }
    virtual bool getTorqueErrorRaw(int j, double *err)
    {
        CD_INFO("\n");
        return true;
    }
    virtual bool getTorqueErrorsRaw(double *errs)
    {
        CD_ERROR("\n");
        return false;
    }
    virtual bool getTorquePidOutputRaw(int j, double *out)
    {
        CD_INFO("\n");
        return true;
    }
    virtual bool getTorquePidOutputsRaw(double *outs)
    {
        CD_ERROR("\n");
        return false;
    }
    virtual bool getTorquePidRaw(int j, yarp::dev::Pid *pid)
    {
        CD_INFO("\n");
        return true;
    }
    virtual bool getTorquePidsRaw(yarp::dev::Pid *pids)
    {
        CD_ERROR("\n");
        return false;
    }
    virtual bool getTorqueErrorLimitRaw(int j, double *limit)
    {
        CD_INFO("\n");
        return true;
    }
    virtual bool getTorqueErrorLimitsRaw(double *limits)
    {
        CD_ERROR("\n");
        return false;
    }
    virtual bool resetTorquePidRaw(int j)
    {
        CD_INFO("\n");
        return true;
    }
    virtual bool disableTorquePidRaw(int j)
    {
        CD_INFO("\n");
        return true;
    }
    virtual bool enableTorquePidRaw(int j)
    {
        CD_INFO("\n");
        return true;
    }
    virtual bool setTorqueOffsetRaw(int j, double v)
    {
        CD_INFO("\n");
        return true;
    }

    //  --------- IVelocityControl Declarations. Implementation in IVelocityControlImpl.cpp ---------
    virtual bool setVelocityModeRaw()
    {
        CD_ERROR("\n");
        return false;
    }
    virtual bool velocityMoveRaw(int j, double sp)
    {
        CD_INFO("\n");
        return true;
    }
    virtual bool velocityMoveRaw(const double *sp)
    {
        CD_ERROR("\n");
        return false;
    }

protected:

    //  --------- Implementation in FakeJoint.cpp ---------
    /**
     * Write message to the CAN buffer.
     * @param cob Message's COB
     * @param len Data field length
     * @param msgData Data to send
     * @return true/false on success/failure.
     */
    bool send(uint32_t cob, uint16_t len, uint8_t * msgData);


    /** pt-related **/
    int ptPointCounter;
    yarp::os::Semaphore ptBuffer;
    bool ptMovementDone;

    bool targetReached;

    int canId;

    ICanBusHico *canDevicePtr;

    double max, min, maxVel, minVel, refAcceleration, refSpeed, tr, targetPosition;

    double lastUsage;

    double encoder;
    uint32_t encoderTimestamp;
    yarp::os::Semaphore encoderReady;

    /** A helper function to display CAN messages. */
    std::string msgToStr(can_msg* message);
    std::string msgToStr(uint32_t cob, uint16_t len, uint8_t * msgData);

    int16_t ptModeMs;  //-- [ms]

    //-- Semaphores
    yarp::os::Semaphore targetPositionSemaphore;
};

}  // namespace teo

#endif  // __FAKE_JOINT__

