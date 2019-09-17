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
                    public yarp::dev::IEncodersTimedRaw,
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

    //  ---------- IEncodersRaw Declarations. Implementation in IEncodersRawImpl.cpp ----------
    virtual bool getAxes(int *ax);
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

    int cuiTimeout;
    int canId;

    CanSenderDelegate * sender;

    double tr;
    double encoder;
    double encoderTimestamp;
    yarp::os::Semaphore encoderReady;
    bool firstHasReached;
};

} // namespace roboticslab

#endif  // __CUI_ABSOLUTE__
