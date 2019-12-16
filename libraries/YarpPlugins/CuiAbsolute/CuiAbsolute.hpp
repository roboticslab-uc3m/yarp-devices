// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __CUI_ABSOLUTE_HPP__
#define __CUI_ABSOLUTE_HPP__

#include <cstdint>

#include <mutex>
#include <string>

#include <yarp/conf/numeric.h>

#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/IEncodersTimed.h>

#include "ICanBusSharer.hpp"
#include "StateObserver.hpp"

#define CHECK_JOINT(j) do { int ax; if (getAxes(&ax), (j) != ax - 1) return false; } while (0)

#define DEFAULT_TIMEOUT 0.1 // [s]
#define DEFAULT_MAX_RETRIES 5

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
        : canId(0), timeout(0.0), maxRetries(0), retry(0), reverse(false),
          cuiMode(CuiMode::OFF), pushDelay(0),
          encoder(), encoderTimestamp(0.0),
          sender(nullptr), pushStateObserver(nullptr), pollStateObserver(nullptr)
    { }

    //  --------- DeviceDriver declarations. Implementation in DeviceDriverImpl.cpp ---------

    virtual bool open(yarp::os::Searchable & config) override;
    virtual bool close() override;

    //  --------- ICanBusSharer declarations. Implementation in ICanBusSharerImpl.cpp ---------

    virtual unsigned int getId() override;
    virtual bool interpretMessage(const yarp::dev::CanMessage & message) override;
    virtual bool initialize() override;
    virtual bool finalize() override;
    virtual bool registerSender(CanSenderDelegate * sender) override;

    //  ---------- IEncodersRaw declarations. Implementation in IEncodersRawImpl.cpp ----------

    virtual bool getAxes(int * ax) override;
    virtual bool resetEncoderRaw(int j) override;
    virtual bool resetEncodersRaw() override;
    virtual bool setEncoderRaw(int j, double val) override;
    virtual bool setEncodersRaw(const double * vals) override;
    virtual bool getEncoderRaw(int j, double * v) override;
    virtual bool getEncodersRaw(double * encs) override;
    virtual bool getEncoderSpeedRaw(int j, double * sp) override;
    virtual bool getEncoderSpeedsRaw(double * spds) override;
    virtual bool getEncoderAccelerationRaw(int j, double * spds) override;
    virtual bool getEncoderAccelerationsRaw(double * accs) override;

    //  ---------- IEncodersTimedRaw declarations. Implementation in IEncodersRawImpl.cpp ----------

    virtual bool getEncodersTimedRaw(double * encs, double * time) override;
    virtual bool getEncoderTimedRaw(int j, double * encs, double * time) override;

private:

    typedef yarp::conf::float32_t encoder_t;

    enum class CuiMode { PUSH, PULL, OFF };
    enum class CuiCommand : std::uint8_t { PUSH_START = 1, PUSH_STOP = 2, POLL = 3 };

    bool performRequest(const std::string & name, unsigned int len, const std::uint8_t * msgData, encoder_t * resp = nullptr);
    bool startPushMode();
    bool stopPushMode();
    bool pollEncoderRead(encoder_t * enc);
    void normalize(encoder_t * v);

    unsigned int canId;
    double timeout;
    int maxRetries;
    int retry;
    bool reverse;

    CuiMode cuiMode;
    std::uint8_t pushDelay;

    encoder_t encoder;
    double encoderTimestamp;

    CanSenderDelegate * sender;
    StateObserver * pushStateObserver;
    TypedStateObserver<encoder_t> * pollStateObserver;

    mutable std::mutex mutex;
};

} // namespace roboticslab

#endif // __CUI_ABSOLUTE_HPP__
