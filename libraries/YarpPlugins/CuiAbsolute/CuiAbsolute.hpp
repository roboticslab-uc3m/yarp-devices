// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __CUI_ABSOLUTE_HPP__
#define __CUI_ABSOLUTE_HPP__

#include <cstddef>
#include <cstdint>

#include <mutex>
#include <string>

#include <yarp/conf/numeric.h>

#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/IEncodersTimed.h>

#include "ICanBusSharer.hpp"
#include "StateObserver.hpp"

#define CHECK_JOINT(j) do { int ax; if (getAxes(&ax), (j) != ax - 1) return false; } while (0)

#define DEFAULT_MODE "push"
#define DEFAULT_TIMEOUT 1.0 // [s]

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
        : canId(0), timeout(0.0), reverse(false),
          cuiMode(CuiMode::OFF), pushDelay(0),
          encoder(), encoderTimestamp(0.0),
          sender(nullptr), pushStateObserver(nullptr), pollStateObserver(nullptr)
    { }

    //  --------- DeviceDriver Declarations. Implementation in DeviceDriverImpl.cpp ---------
    virtual bool open(yarp::os::Searchable& config);
    virtual bool close();

    //  --------- ICanBusSharer Declarations. Implementation in ICanBusSharerImpl.cpp ---------
    virtual unsigned int getId();
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

    enum class CuiMode { PUSH, PULL, OFF };
    enum class CuiCommand : std::uint8_t { PUSH_START = 1, PUSH_STOP = 2, POLL = 3 };

    bool performRequest(const std::string & name, std::size_t len, const std::uint8_t * msgData, double * resp = nullptr);
    bool startPushMode();
    bool stopPushMode();
    bool pollEncoderRead(double * enc);

    unsigned int canId;
    double timeout;
    bool reverse;

    CuiMode cuiMode;
    std::uint8_t pushDelay;

    yarp::conf::float32_t encoder;
    double encoderTimestamp;

    CanSenderDelegate * sender;
    StateObserver * pushStateObserver;
    TypedStateObserver<double> * pollStateObserver;

    mutable std::mutex mutex;
};

} // namespace roboticslab

#endif // __CUI_ABSOLUTE_HPP__
