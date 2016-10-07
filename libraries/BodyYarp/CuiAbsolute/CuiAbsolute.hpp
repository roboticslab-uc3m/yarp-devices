// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __CUI_ABSOLUTE__
#define __CUI_ABSOLUTE__

#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <sstream>
#include <math.h>

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
 * \defgroup CuiAbsolute
 * @brief Contains teo::CuiAbsolute.
 */

/**
* @ingroup CuiAbsolute
* @brief Implementation for the Cui Absolute Encoder custom UC3M circuit as a single CAN bus joint (controlboard raw interfaces).
*
*/
// Note: IEncodersTimedRaw inherits from IEncodersRaw
// -- Nota: Definimos todas las funciones de los Drivers en los CuiAbsolute debido a que hereda todas las clases siguientes.
//          Al final definiremos una función auxiliar que será la que utilicemos para enviar mensajes al PIC.
class CuiAbsolute : public yarp::dev::DeviceDriver, public yarp::dev::IControlLimitsRaw, public yarp::dev::IControlModeRaw, public yarp::dev::IEncodersTimedRaw,
    public yarp::dev::IPositionControlRaw, public yarp::dev::IPositionDirectRaw, public yarp::dev::IVelocityControlRaw, public yarp::dev::ITorqueControlRaw,
    public ICanBusSharer
{

public:

    CuiAbsolute()
    {
        canDevicePtr = 0;
        firstHasReached = false;
    }

    bool HasFirstReached(){
        return firstHasReached;
    }

    //  --------- DeviceDriver Declarations. Implementation in CuiAbsolute.cpp ---------
    virtual bool open(yarp::os::Searchable& config);
    virtual bool close();

    //  --------- ICanBusSharer Declarations. Implementation in CuiAbsolute.cpp ---------
    virtual bool setCanBusPtr(CanBusHico *canDevicePtr);
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

    // -- Auxiliary functions: send data to PIC of Cui

    bool startContinuousPublishing(uint8_t time);
    bool startPullPublishing();
    bool stopPublishingMessages();    


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

    /** pt-related **/
    int ptPointCounter;
    yarp::os::Semaphore ptBuffer;
    bool ptMovementDone;

    bool targetReached;

    int canId;

    CanBusHico *canDevicePtr;

    double max, min, refAcceleration, refSpeed, tr;

    double lastUsage;


    //-- Encoder stuff
    double encoder;
    uint32_t encoderTimestamp;
    yarp::os::Semaphore encoderReady;
    bool firstHasReached;

    /** A helper function to display CAN messages. */
    std::string msgToStr(can_msg* message);
    std::string msgToStr(uint32_t cob, uint16_t len, uint8_t * msgData);

    int16_t ptModeMs;  //-- [ms]
};

}  // namespace teo

#endif  // __CUI_ABSOLUTE__

