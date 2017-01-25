// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __TECHNOSOFT_IPOS__
#define __TECHNOSOFT_IPOS__

#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <sstream>
#include <math.h>  // roundf

//#define CD_FULL_FILE  //-- Can be globally managed from father CMake. Good for debugging with polymorphism.
//#define CD_HIDE_DEBUG  //-- Can be globally managed from father CMake.
//#define CD_HIDE_SUCCESS  //-- Can be globally managed from father CMake.
//#define CD_HIDE_INFO  //-- Can be globally managed from father CMake.
//#define CD_HIDE_WARNING  //-- Can be globally managed from father CMake.
//#define CD_HIDE_ERROR  //-- Can be globally managed from father CMake.
#include "ColorDebug.hpp"
#include "ICanBusSharer.h"
#include "ITechnosoftIpos.h"

namespace teo
{

/**
 * @ingroup BodyYarp
 * \defgroup TechnosoftIpos
 * @brief Contains teo::TechnosoftIpos.
 */

/**
* @ingroup TechnosoftIpos
* @brief Implementation for the Technosoft iPOS as a single CAN bus joint (controlboard raw interfaces).
*
*/
// Note: IEncodersTimedRaw inherits from IEncodersRaw
class TechnosoftIpos : public yarp::dev::DeviceDriver, public yarp::dev::IControlLimitsRaw, public yarp::dev::IControlModeRaw, public yarp::dev::IEncodersTimedRaw,
    public yarp::dev::IPositionControlRaw, public yarp::dev::IPositionDirectRaw, public yarp::dev::IVelocityControlRaw, public yarp::dev::ITorqueControlRaw,
    public ICanBusSharer, public ITechnosoftIpos
{

public:

    TechnosoftIpos()
    {
        canDevicePtr = 0;
        iEncodersTimedRawExternal = 0;
    }

    //  --------- DeviceDriver Declarations. Implementation in TechnosoftIpos.cpp ---------
    virtual bool open(yarp::os::Searchable& config);
    virtual bool close();

    //  --------- ICanBusSharer Declarations. Implementation in TechnosoftIpos.cpp ---------
    virtual bool setCanBusPtr(ICanBusHico *canDevicePtr);
    virtual bool setIEncodersTimedRawExternal(IEncodersTimedRaw * iEncodersTimedRaw); // -- ??
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
    /** reset node */
    virtual bool resetNode(int id);
    /** reset all nodes */
    virtual bool resetNodes();
    /** reset communications */
    virtual bool resetCommunication();

    //  --------- IControlLimitsRaw Declarations. Implementation in IControlLimitsRawImpl.cpp ---------
    virtual bool setLimitsRaw(int axis, double min, double max);
    virtual bool getLimitsRaw(int axis, double *min, double *max);
    //-- Auxiliary functions of setLimitsRaw
    bool setMinLimitRaw(double min);
    bool setMaxLimitRaw(double max);

    //  --------- IControlModeRaw Declarations. Implementation in IControlModeRawImpl.cpp ---------
    virtual bool setPositionModeRaw(int j);
    virtual bool setVelocityModeRaw(int j);
    virtual bool setTorqueModeRaw(int j);
    //-- Auxiliary functions (splitted) of setTorqueModeRaw
    bool setTorqueModeRaw1();
    bool setTorqueModeRaw2();
    bool setTorqueModeRaw3();

    virtual bool setImpedancePositionModeRaw(int j);
    virtual bool setImpedanceVelocityModeRaw(int j);
    virtual bool setOpenLoopModeRaw(int j);
    virtual bool getControlModeRaw(int j, int *mode);
    //-- Auxiliary functions (splitted) of getControlModeRaw
    bool getControlModeRaw1();
    bool getControlModeRaw2();
    bool getControlModeRaw3();
    bool getControlModeRaw4();

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
    virtual bool getAxes(int *ax)
    {
        *ax = 1;
        return true;
    }
    virtual bool setPositionModeRaw()
    {
        return setPositionModeRaw(0);
    }
    virtual bool positionMoveRaw(int j, double ref);
    virtual bool positionMoveRaw(const double *refs)
    {
        CD_ERROR("\n");
        return false;
    }
    virtual bool relativeMoveRaw(int j, double delta);
    virtual bool relativeMoveRaw(const double *deltas)
    {
        CD_ERROR("\n");
        return false;
    }
    virtual bool checkMotionDoneRaw(int j, bool *flag);
    virtual bool checkMotionDoneRaw(bool *flag)
    {
        CD_ERROR("\n");
        return false;
    }
    virtual bool setRefSpeedRaw(int j, double sp);
    virtual bool setRefSpeedsRaw(const double *spds)
    {
        CD_ERROR("\n");
        return false;
    }
    virtual bool setRefAccelerationRaw(int j, double acc);
    virtual bool setRefAccelerationsRaw(const double *accs)
    {
        CD_ERROR("\n");
        return false;
    }
    virtual bool getRefSpeedRaw(int j, double *ref);
    virtual bool getRefSpeedsRaw(double *spds)
    {
        CD_ERROR("\n");
        return false;
    }
    virtual bool getRefAccelerationRaw(int j, double *acc);
    virtual bool getRefAccelerationsRaw(double *accs)
    {
        CD_ERROR("\n");
        return false;
    }
    virtual bool stopRaw(int j);
    virtual bool stopRaw()
    {
        CD_ERROR("\n");
        return false;
    }

    // ------- IPositionDirectRaw declarations. Implementation in IPositionDirectRawImpl.cpp -------
    virtual bool setPositionDirectModeRaw();
    virtual bool setPositionRaw(int j, double ref);
    virtual bool setPositionsRaw(const int n_joint, const int *joints, double *refs);
    virtual bool setPositionsRaw(const double *refs);

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
    virtual bool getRefTorqueRaw(int j, double *t);
    virtual bool setRefTorquesRaw(const double *t)
    {
        CD_ERROR("\n");
        return false;
    }
    virtual bool setRefTorqueRaw(int j, double t);
    virtual bool getBemfParamRaw(int j, double *bemf);
    virtual bool setBemfParamRaw(int j, double bemf);
    virtual bool setTorquePidRaw(int j, const yarp::dev::Pid &pid);
    virtual bool getTorqueRaw(int j, double *t);
    virtual bool getTorquesRaw(double *t)
    {
        CD_ERROR("\n");
        return false;
    }
    virtual bool getTorqueRangeRaw(int j, double *min, double *max);
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
    virtual bool setTorqueErrorLimitRaw(int j, double limit);
    virtual bool setTorqueErrorLimitsRaw(const double *limits)
    {
        CD_ERROR("\n");
        return false;
    }
    virtual bool getTorqueErrorRaw(int j, double *err);
    virtual bool getTorqueErrorsRaw(double *errs)
    {
        CD_ERROR("\n");
        return false;
    }
    virtual bool getTorquePidOutputRaw(int j, double *out);
    virtual bool getTorquePidOutputsRaw(double *outs)
    {
        CD_ERROR("\n");
        return false;
    }
    virtual bool getTorquePidRaw(int j, yarp::dev::Pid *pid);
    virtual bool getTorquePidsRaw(yarp::dev::Pid *pids)
    {
        CD_ERROR("\n");
        return false;
    }
    virtual bool getTorqueErrorLimitRaw(int j, double *limit);
    virtual bool getTorqueErrorLimitsRaw(double *limits)
    {
        CD_ERROR("\n");
        return false;
    }
    virtual bool resetTorquePidRaw(int j);
    virtual bool disableTorquePidRaw(int j);
    virtual bool enableTorquePidRaw(int j);
    virtual bool setTorqueOffsetRaw(int j, double v);

    //  --------- IVelocityControl Declarations. Implementation in IVelocityControlImpl.cpp ---------
    virtual bool setVelocityModeRaw()
    {
        CD_ERROR("\n");
        return false;
    }
    virtual bool velocityMoveRaw(int j, double sp);
    virtual bool velocityMoveRaw(const double *sp)
    {
        CD_ERROR("\n");
        return false;
    }

protected:

    //  --------- Implementation in TechnosoftIpos.cpp ---------

    //-- CAN bus stuff
    /**
     * Write message to the CAN buffer.
     * @param cob Message's COB
     * @param len Data field length
     * @param msgData Data to send
     * @return true/false on success/failure.
     */
    bool send(uint32_t cob, uint16_t len, uint8_t * msgData);

    /** A helper function to display CAN messages. */
    std::string msgToStr(can_msg* message);
    std::string msgToStr(uint32_t cob, uint16_t len, uint8_t * msgData);

    int canId;
    ICanBusHico *canDevicePtr;
    double lastUsage;

    //-- Encoder stuff
    double encoder;
    uint32_t encoderTimestamp;
    yarp::os::Semaphore encoderReady;
    yarp::dev::IEncodersTimedRaw* iEncodersTimedRawExternal;

    //-- Mode stuff
    int getMode;
    yarp::os::Semaphore getModeReady;

    bool targetReached;
    yarp::os::Semaphore targetReachedReady;

    //-- Torque stuff
    double getTorque;
    yarp::os::Semaphore getTorqueReady;

    //-- Init stuff
    int getSwitchOn;
    yarp::os::Semaphore getSwitchOnReady;

    int getEnable;
    yarp::os::Semaphore getEnableReady;

    //-- PT stuff
    int16_t ptModeMs;  //-- [ms]
    int ptPointCounter;
    bool ptMovementDone;
    yarp::os::Semaphore ptBuffer;

    //-- More internal parameter stuff
    double max, min, refAcceleration, refSpeed, tr, k;

};

}  // namespace teo

#endif  // __TECHNOSOFT_IPOS__

