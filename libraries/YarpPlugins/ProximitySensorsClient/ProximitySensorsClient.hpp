// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __PROXIMITY_SENSORS_CLIENT_HPP__
#define __PROXIMITY_SENSORS_CLIENT_HPP__

#include <yarp/os/all.h>
#include <yarp/dev/Drivers.h>
#include <yarp/dev/PolyDriver.h>

#include "IProximitySensors.h"

#define DEFAULT_LOCAL "/sensor_reader"
#define DEFAULT_REMOTE "/serial/out"

#define DEFAULT_PORTMONITOR_TYPE "lua"
#define DEFAULT_PORTMONITOR_CONTEXT "sensors"
#define DEFAULT_PORTMONITOR_FILE "amor_sensors_modifier"

#define DEFAULT_THRESHOLD_GRIPPER 50
#define DEFAULT_THRESHOLD_ALERT_HIGH 800
#define DEFAULT_THRESHOLD_ALERT_LOW 100

namespace roboticslab
{

/**
 * @ingroup BodyYarp
 * \defgroup ProximitySensorsClient
 *
 * @brief Contains roboticslab::ProximitySensorsClient.
 */

/**
 * @ingroup ProximitySensorsClient
 * @brief The ProximitySensorsClient class implements IProximitySensors.
 */
class ProximitySensorsClient : public yarp::dev::DeviceDriver, public IProximitySensors
{
public:

    ProximitySensorsClient() : alert(ZERO),
                               gripper(false),
                               thresholdGripper(DEFAULT_THRESHOLD_GRIPPER),
                               thresholdAlertHigh(DEFAULT_THRESHOLD_ALERT_HIGH),
                               thresholdAlertLow(DEFAULT_THRESHOLD_ALERT_LOW)
    {}

    // -------- IProximitySensors declarations. Implementation in IProximitySensorsImpl.cpp --------

    virtual alert_level getAlertLevel();
    virtual bool hasTarget();

    // -------- DeviceDriver declarations. Implementation in IDeviceImpl.cpp --------

    /**
     * Open the DeviceDriver.
     * @param config is a list of parameters for the device.
     * Which parameters are effective for your device can vary.
     * See \ref dev_examples "device invocation examples".
     * If there is no example for your device,
     * you can run the "yarpdev" program with the verbose flag
     * set to probe what parameters the device is checking.
     * If that fails too,
     * you'll need to read the source code (please nag one of the
     * yarp developers to add documentation for your device).
     * @return true/false upon success/failure
     */
    virtual bool open(yarp::os::Searchable& config);

    /**
     * Close the DeviceDriver.
     * @return true/false on success/failure.
     */
    virtual bool close();

protected:

    class SensorReader : public yarp::os::BufferedPort<yarp::os::Bottle>
    {
    public:
        SensorReader() : sens(NULL) {}
        virtual void onRead(yarp::os::Bottle& b);
        void setReference(ProximitySensorsClient * sens)
        {
            this->sens = sens;
        }
    private:
        ProximitySensorsClient * sens;
    };

    SensorReader sr;

    alert_level alert;
    bool gripper;

    yarp::os::Mutex alertMutex, gripperMutex;

    int thresholdGripper, thresholdAlertHigh, thresholdAlertLow;
};

}  // namespace roboticslab

#endif  // __PROXIMITY_SENSORS_CLIENT_HPP__
