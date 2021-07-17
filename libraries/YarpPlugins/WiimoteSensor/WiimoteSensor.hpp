// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __WIIMOTE_SENSOR_HPP__
#define __WIIMOTE_SENSOR_HPP__

#include <mutex>

#include <poll.h>
#include <xwiimote.h>

#include <yarp/os/Thread.h>

#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/IAnalogSensor.h>

namespace roboticslab
{

/**
 * @ingroup YarpPlugins
 * @defgroup WiimoteSensor
 * @brief Contains roboticslab::WiimoteSensor.
 */

/**
 * @ingroup WiimoteSensor
 * @brief Holds key state data from input events.
 */
struct WiimoteEventData
{
    WiimoteEventData()
        : accelX(0), accelY(0), accelZ(0),
          buttonA(false), buttonB(false),
          button1(false), button2(false)
    {}

    int accelX;
    int accelY;
    int accelZ;

    bool buttonA;
    bool buttonB;

    bool button1;
    bool button2;
};

/**
 * @ingroup WiimoteSensor
 * @brief Thread that listens to Wiimote events.
 */
class WiimoteDispatcherThread : public yarp::os::Thread
{
public:

    //! @brief Constructor.
    WiimoteDispatcherThread() : iface(nullptr)
    { }

    //! @brief Called just before a new thread starts.
    void beforeStart() override;

    //! @brief Main body of the new thread.
    void run() override;

    //! @brief Set pointer to @ref xwii_iface.
    void setInterfacePointer(struct xwii_iface * iface)
    {
        this->iface = iface;
    }

    //! @brief Retrieve event data object.
    WiimoteEventData getEventData() const;

private:

    struct xwii_iface * iface;
    struct xwii_event event;

    WiimoteEventData eventData;
    mutable std::mutex eventDataMutex;

    struct pollfd fds[2];
    int fds_num;
};

/**
 * @ingroup WiimoteSensor
 * @brief Implementation for the Wiimote controller.
 */
class WiimoteSensor : public yarp::dev::DeviceDriver,
                      public yarp::dev::IAnalogSensor
{
public:

    WiimoteSensor()
        : iface(nullptr),
          calibZeroX(0), calibZeroY(0), calibZeroZ(0),
          calibOneX(0), calibOneY(0), calibOneZ(0)
    { }

    ~WiimoteSensor() override
    { close(); }

    //  --------- DeviceDriver Declarations. Implementation in DeviceDriverImpl.cpp ---------
    bool open(yarp::os::Searchable& config) override;
    bool close() override;

    //  --------- IAnalogSensor Declarations. Implementation in IAnalogSensorImpl.cpp ---------
    /**
     * Read a vector from the sensor.
     * @param out a vector containing the sensor's last readings.
     * @return AS_OK or return code. AS_TIMEOUT if the sensor timed-out.
     */
    int read(yarp::sig::Vector &out) override;

    /**
     * Check the state value of a given channel.
     * @param ch channel number.
     * @return status.
     */
    int getState(int ch) override;

    /**
     * Get the number of channels of the sensor.
     * @return number of channels (0 in case of errors).
     */
    int getChannels() override;

    /**
     * Calibrates the whole sensor.
     * @return status.
     */
    int calibrateSensor() override;

    /**
     * Calibrates the whole sensor, using an vector of calibration values.
     * @param value a vector of calibration values.
     * @return status.
     */
    int calibrateSensor(const yarp::sig::Vector& value) override;

    /**
     * Calibrates one single channel.
     * @param ch channel number.
     * @return status.
     */
    int calibrateChannel(int ch) override;

    /**
     * Calibrates one single channel, using a calibration value.
     * @param ch channel number.
     * @param value calibration value.
     * @return status.
     */
    int calibrateChannel(int ch, double value) override;

private:

    static char * getDevicePath(int id);

    struct xwii_iface * iface;

    int calibZeroX, calibZeroY, calibZeroZ;
    int calibOneX, calibOneY, calibOneZ;
    bool yawActive;

    WiimoteDispatcherThread dispatcherThread;
};

} // namespace roboticslab

#endif // __WIIMOTE_SENSOR_HPP__
