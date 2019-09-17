// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __CUI_ABSOLUTE_HPP__
#define __CUI_ABSOLUTE_HPP__

#include <mutex>

#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/IEncodersTimed.h>

#include "ICanBusSharer.hpp"

#define CHECK_JOINT(j) do { int ax; if (getAxes(&ax), (j) != ax - 1) return false; } while (0)

namespace roboticslab
{

/**
 * @ingroup YarpPlugins
 * \defgroup CuiAbsolute
 * @brief Contains roboticslab::CuiAbsolute.
 */

/**
 * @ingroup CuiAbsolute
 * @brief Implementation for the Cui Absolute Encoder custom UC3M circuit as a single
 * CAN bus joint (controlboard raw interfaces).
 */
class CuiAbsolute : public yarp::dev::DeviceDriver,
                    public yarp::dev::IEncodersTimedRaw,
                    public ICanBusSharer
{
public:

    CuiAbsolute()
        : canId(0), cuiTimeout(0.0), reverse(false), encoder(0.0), encoderTimestamp(0.0),
          firstHasReached(false), sender(nullptr)
    { }

    //  --------- DeviceDriver Declarations. Implementation in DeviceDriverImpl.cpp ---------
    virtual bool open(yarp::os::Searchable& config);
    virtual bool close();

    //  --------- ICanBusSharer Declarations. Implementation in ICanBusSharerImpl.cpp ---------
    virtual bool setIEncodersTimedRawExternal(IEncodersTimedRaw * iEncodersTimedRaw);
    virtual bool interpretMessage(const yarp::dev::CanMessage & message);
    virtual bool initialize();
    virtual bool start();
    virtual bool readyToSwitchOn();
    virtual bool switchOn();
    virtual bool enable();
    virtual bool recoverFromError();
    virtual bool registerSender(CanSenderDelegate * sender);

    //  ---------- IEncodersRaw Declarations. Implementation in IEncodersTimedRawImpl.cpp ----------
    virtual bool getAxes(int * ax);
    virtual bool resetEncoderRaw(int j);
    virtual bool resetEncodersRaw();
    virtual bool setEncoderRaw(int j, double val);
    virtual bool setEncodersRaw(const double * vals);
    virtual bool getEncoderRaw(int j, double * v);
    virtual bool getEncodersRaw(double * encs);
    virtual bool getEncoderSpeedRaw(int j, double * sp);
    virtual bool getEncoderSpeedsRaw(double * spds);
    virtual bool getEncoderAccelerationRaw(int j, double * spds);
    virtual bool getEncoderAccelerationsRaw(double * accs);

    //  ---------- IEncodersTimedRaw Declarations. Implementation in IEncodersTimedRawImpl.cpp ----------
    virtual bool getEncodersTimedRaw(double * encs, double * time);
    virtual bool getEncoderTimedRaw(int j, double * encs, double * time);

private:

    bool send(std::uint16_t len, std::uint8_t * msgData);
    bool startContinuousPublishing(uint8_t time);
    bool startPullPublishing();
    bool stopPublishingMessages();

    unsigned int canId;
    double cuiTimeout;
    bool reverse;

    double encoder;
    double encoderTimestamp;
    bool firstHasReached;

    CanSenderDelegate * sender;

    mutable std::mutex mutex;
};

} // namespace roboticslab

#endif // __CUI_ABSOLUTE_HPP__
