// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __LEAP_MOTION_SENSOR__
#define __LEAP_MOTION_SENSOR__

#include <Leap.h>

#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/IAnalogSensor.h>

namespace roboticslab
{

/**
 * @ingroup YarpPlugins
 * @defgroup LeapMotionSensor
 * @brief Contains roboticslab::LeapMotionSensor.
 */

 /**
 * @ingroup LeapMotionSensor
 * @brief Implementation for the LeapMotionSensor controller.
 *
 * Launch as in:
@verbatim
yarpdev --device LeapMotionSensor --period 5 --name /leapmotion
@endverbatim
 */
class LeapMotionSensor : public yarp::dev::DeviceDriver, public yarp::dev::IAnalogSensor
{
public:

    LeapMotionSensor() : controller(NULL)
    { }

    ~LeapMotionSensor()
    { close(); }

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

protected:

    Leap::Controller * controller;
};

}  // namespace roboticslab

#endif  // __LEAP_MOTION_SENSOR__
