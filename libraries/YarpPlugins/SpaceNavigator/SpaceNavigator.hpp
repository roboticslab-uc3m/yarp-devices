// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __SPACE_NAVIGATOR_HPP__
#define __SPACE_NAVIGATOR_HPP__

#include <spnav.h>

#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/IAnalogSensor.h>

namespace roboticslab
{

/**
 * @ingroup YarpPlugins
 * @defgroup SpaceNavigator
 * @brief Contains roboticslab::SpaceNavigator.
 */

 /**
 * @ingroup SpaceNavigator
 * @brief Implementation for the SpaceNavigator 3D mouse.
 *
 * Launch as in:
@verbatim
yarpdev --device SpaceNavigator --period 5 --name /spacenavigator
@endverbatim
 * You can split mouse and button output into separate channels with:
@verbatim
yarpdev --device SpaceNavigator --period 5 --name /spacenavigator --ports "(mouse:o buttons:o)" --channels 8 --mouse:o 0 5 0 5 --buttons:o 6 7 0 1
@endverbatim
 */
class SpaceNavigator : public yarp::dev::DeviceDriver,
                       public yarp::dev::IAnalogSensor
{
public:

    ~SpaceNavigator() override
    { close(); }

    //  --------- DeviceDriver Declarations. Implementation in DeviceDriverImpl.cpp ---------
    bool open(yarp::os::Searchable & config) override;
    bool close() override;

    //  --------- IAnalogSensor Declarations. Implementation in IAnalogSensorImpl.cpp ---------
    /**
     * Read a vector from the sensor.
     * @param out a vector containing the sensor's last readings.
     * @return AS_OK or return code. AS_TIMEOUT if the sensor timed-out.
     */
    int read(yarp::sig::Vector & out) override;

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
    int calibrateSensor(const yarp::sig::Vector & value) override;

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

    double dx {0.0};
    double dy {0.0};
    double dz {0.0};

    double droll {0.0};
    double dpitch {0.0};
    double dyaw {0.0};

    int button1 {0};
    int button2 {0};

    unsigned int noDataCounter {0};
    double deadband {0.0};
};

} // namespace roboticslab

#endif // __SPACE_NAVIGATOR_HPP__
