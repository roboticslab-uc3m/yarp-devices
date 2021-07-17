// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __LEAP_MOTION_SENSOR_HPP__
#define __LEAP_MOTION_SENSOR_HPP__

#include <Leap.h>

#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/IAnalogSensor.h>

#include <yarp/sig/Vector.h>

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
class LeapMotionSensor : public yarp::dev::DeviceDriver,
                         public yarp::dev::IAnalogSensor
{
public:

    LeapMotionSensor() : controller(nullptr)
    { }

    ~LeapMotionSensor() override
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

protected:

    Leap::Controller * controller;
    int32_t currentHandId;
    yarp::sig::Vector lastValidData;
};

} // namespace roboticslab

#endif // __LEAP_MOTION_SENSOR_HPP__
