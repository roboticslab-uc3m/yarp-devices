// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __WIIMOTE_SENSOR_HPP__
#define __WIIMOTE_SENSOR_HPP__

#include <xwiimote.h>

#include <yarp/os/Thread.h>
#include <yarp/os/Mutex.h>
#include <yarp/dev/IAnalogSensor.h>

#define DEFAULT_DEVICE 1

#define DEFAULT_CALIB_ZERO_X -30
#define DEFAULT_CALIB_ZERO_Y -22
#define DEFAULT_CALIB_ZERO_Z 72

#define DEFAULT_CALIB_ONE_X 69
#define DEFAULT_CALIB_ONE_Y -123
#define DEFAULT_CALIB_ONE_Z -25

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
    WiimoteDispatcherThread() : iface(NULL)
    {}

    //! @brief Called just before a new thread starts.
    virtual void beforeStart();

    //! @brief Main body of the new thread.
    virtual void run();

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
    mutable yarp::os::Mutex eventDataMutex;
};

/**
 * @ingroup WiimoteSensor
 * @brief Implementation for the Wiimote controller.
 */
class WiimoteSensor : public yarp::dev::DeviceDriver, public yarp::dev::IAnalogSensor
{
public:

    WiimoteSensor()
        : iface(NULL),
          calibZeroX(0), calibZeroY(0), calibZeroZ(0),
          calibOneX(0), calibOneY(0), calibOneZ(0)
    {}

    //  --------- DeviceDriver Declarations. Implementation in DeviceDriverImpl.cpp ---------
    virtual bool open(yarp::os::Searchable& config);
    virtual bool close();

    //  --------- IAnalogSensor Declarations. Implementation in IAnalogSensorImpl.cpp ---------
    /**
     * Read a vector from the sensor.
     * @param out a vector containing the sensor's last readings.
     * @return AS_OK or return code. AS_TIMEOUT if the sensor timed-out.
     */
    virtual int read(yarp::sig::Vector &out);

    /**
     * Check the state value of a given channel.
     * @param ch channel number.
     * @return status.
     */
    virtual int getState(int ch);

    /**
     * Get the number of channels of the sensor.
     * @return number of channels (0 in case of errors).
     */
    virtual int getChannels();

    /**
     * Calibrates the whole sensor.
     * @return status.
     */
    virtual int calibrateSensor();

    /**
     * Calibrates the whole sensor, using an vector of calibration values.
     * @param value a vector of calibration values.
     * @return status.
     */
    virtual int calibrateSensor(const yarp::sig::Vector& value);

    /**
     * Calibrates one single channel.
     * @param ch channel number.
     * @return status.
     */
    virtual int calibrateChannel(int ch);

    /**
     * Calibrates one single channel, using a calibration value.
     * @param ch channel number.
     * @param value calibration value.
     * @return status.
     */
    virtual int calibrateChannel(int ch, double value);

private:

    static char * getDevicePath(int id);

    struct xwii_iface * iface;

    int calibZeroX, calibZeroY, calibZeroZ;
    int calibOneX, calibOneY, calibOneZ;

    WiimoteDispatcherThread dispatcherThread;
};

}  // namespace roboticslab

#endif  // __WIIMOTE_SENSOR_HPP__
