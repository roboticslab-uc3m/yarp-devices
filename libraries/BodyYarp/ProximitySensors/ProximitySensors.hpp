// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __PROXIMITY_SENSORS_HPP__
#define __PROXIMITY_SENSORS_HPP__

#include <yarp/os/all.h>
#include <yarp/dev/Drivers.h>
#include <yarp/dev/PolyDriver.h>

#include "IProximitySensors.h"

#define DEFAULT_LOCAL "/sensor_reader"
#define DEFAULT_REMOTE "/serial/out"

#define DEFAULT_PORTMONITOR_TYPE "lua"
#define DEFAULT_PORTMONITOR_CONTEXT "sensors"
#define DEFAULT_PORTMONITOR_FILE "amor_sensors_modifier"

namespace roboticslab
{

/**
 * @ingroup TeoYarp
 * \defgroup ProximitySensors
 *
 * @brief Contains roboticslab::ProximitySensors.
 */

/**
 * @ingroup ProximitySensors
 * @brief The ProximitySensors class implements IProximitySensors.
 */
class ProximitySensors : public yarp::dev::DeviceDriver, public IProximitySensors
{
    public:

        ProximitySensors() : alert(ZERO), gripper(false)
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

        static const int THRESHOLD_GRIPPER;
        static const int THRESHOLD_ALERT;
        static const int THRESHOLD_LOW_ALERT;

    protected:

        class SensorReader : public yarp::os::BufferedPort<yarp::os::Bottle>
        {
        public:
            SensorReader() : sens(NULL) {}
            virtual void onRead(yarp::os::Bottle& b);
            void setReference(ProximitySensors * sens)
            {
                this->sens = sens;
            }
        private:
            ProximitySensors * sens;
        };

        SensorReader sr;
        alert_level alert;
        bool gripper;
        yarp::os::Mutex alertMutex, gripperMutex;

};

}  // namespace roboticslab

#endif  // __PROXIMITY_SENSORS_HPP__


