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

namespace roboticslab
{

/**
 * @ingroup YarpPlugins
 * @defgroup CuiAbsolute
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

    ~CuiAbsolute() override
    { close(); }

    //  --------- DeviceDriver declarations. Implementation in DeviceDriverImpl.cpp ---------

    bool open(yarp::os::Searchable & config) override;
    bool close() override;

    //  --------- ICanBusSharer declarations. Implementation in ICanBusSharerImpl.cpp ---------

    unsigned int getId() override;
    bool notifyMessage(const can_message & message) override;
    bool initialize() override;
    bool finalize() override;
    bool registerSender(CanSenderDelegate * sender) override;
    bool synchronize() override;

    //  ---------- IEncodersRaw declarations. Implementation in IEncodersRawImpl.cpp ----------

    bool getAxes(int * ax) override;
    bool resetEncoderRaw(int j) override;
    bool resetEncodersRaw() override;
    bool setEncoderRaw(int j, double val) override;
    bool setEncodersRaw(const double * vals) override;
    bool getEncoderRaw(int j, double * v) override;
    bool getEncodersRaw(double * encs) override;
    bool getEncoderSpeedRaw(int j, double * sp) override;
    bool getEncoderSpeedsRaw(double * spds) override;
    bool getEncoderAccelerationRaw(int j, double * spds) override;
    bool getEncoderAccelerationsRaw(double * accs) override;

    //  ---------- IEncodersTimedRaw declarations. Implementation in IEncodersRawImpl.cpp ----------

    bool getEncodersTimedRaw(double * encs, double * time) override;
    bool getEncoderTimedRaw(int j, double * encs, double * time) override;

private:

    using encoder_t = yarp::conf::float32_t;

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
