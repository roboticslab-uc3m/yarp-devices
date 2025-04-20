// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __WIIMOTE_SENSOR_HPP__
#define __WIIMOTE_SENSOR_HPP__

#include <mutex>

#include <poll.h>
#include <xwiimote.h>

#include <yarp/os/Thread.h>

#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/IAnalogSensor.h>

#include "WiimoteSensor_ParamsParser.h"

/**
 * @ingroup YarpPlugins
 * @defgroup WiimoteSensor
 * @brief Contains WiimoteSensor.
 */

/**
 * @ingroup WiimoteSensor
 * @brief Holds key state data from input events.
 */
struct WiimoteEventData
{
    int accelX {0};
    int accelY {0};
    int accelZ {0};

    bool buttonA {false};
    bool buttonB {false};

    bool button1 {false};
    bool button2 {false};
};

/**
 * @ingroup WiimoteSensor
 * @brief Thread that listens to Wiimote events.
 */
class WiimoteDispatcherThread : public yarp::os::Thread
{
public:

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

    struct xwii_iface * iface {nullptr};
    struct xwii_event event;

    WiimoteEventData eventData;
    mutable std::mutex eventDataMutex;

    struct pollfd fds[2];
    int fds_num {0};
};

/**
 * @ingroup WiimoteSensor
 * @brief Implementation for the Wiimote controller.
 */
class WiimoteSensor : public yarp::dev::DeviceDriver,
                      public yarp::dev::IAnalogSensor,
                      public WiimoteSensor_ParamsParser
{
public:
    //  --------- DeviceDriver Declarations. Implementation in DeviceDriverImpl.cpp ---------
    bool open(yarp::os::Searchable& config) override;
    bool close() override;

    //  --------- IAnalogSensor Declarations. Implementation in IAnalogSensorImpl.cpp ---------
    int read(yarp::sig::Vector &out) override;
    int getState(int ch) override;
    int getChannels() override;
    int calibrateSensor() override;
    int calibrateSensor(const yarp::sig::Vector& value) override;
    int calibrateChannel(int ch) override;
    int calibrateChannel(int ch, double value) override;

private:
    static char * getDevicePath(int id);

    struct xwii_iface * iface {nullptr};

    bool yawActive {false};

    WiimoteDispatcherThread dispatcherThread;
};

#endif // __WIIMOTE_SENSOR_HPP__
