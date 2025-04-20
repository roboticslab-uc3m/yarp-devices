// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __LEAP_MOTION_SENSOR_HPP__
#define __LEAP_MOTION_SENSOR_HPP__

#include <Leap.h>

#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/IAnalogSensor.h>

#include <yarp/sig/Vector.h>

/**
 * @ingroup YarpPlugins
 * @defgroup LeapMotionSensor
 * @brief Contains LeapMotionSensor.
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
    Leap::Controller * controller {nullptr};
    int32_t currentHandId {0};
    yarp::sig::Vector lastValidData;
};

#endif // __LEAP_MOTION_SENSOR_HPP__
