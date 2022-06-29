// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __JR3_HPP__
#define __JR3_HPP__

#include <string>
#include <vector>

#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/IAnalogSensor.h>
#include <yarp/dev/MultipleAnalogSensorsInterfaces.h>

#include "jr3pci-ioctl.h"

namespace roboticslab
{

/**
 * @ingroup YarpPlugins
 * @defgroup Jr3
 * @brief Contains roboticslab::Jr3.
 */

 /**
 * @ingroup Jr3
 * @brief Implementation for the JR3 sensor. Launch as in: yarpdev --device Jr3 --period 20 --name /jr3:o
 */
class Jr3 : public yarp::dev::DeviceDriver,
            public yarp::dev::IAnalogSensor,
            public yarp::dev::ISixAxisForceTorqueSensors
{
public:
    Jr3() : fd(0)
    { }

    ~Jr3() override
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

    //  --------- ISixAxisForceTorqueSensors Declarations. Implementation in ISixAxisForceTorqueSensorsImpl.cpp ---------
    std::size_t getNrOfSixAxisForceTorqueSensors() const override;
    yarp::dev::MAS_status getSixAxisForceTorqueSensorStatus(std::size_t sens_index) const override;
    bool getSixAxisForceTorqueSensorName(std::size_t sens_index, std::string & name) const override;
    bool getSixAxisForceTorqueSensorFrameName(std::size_t sens_index, std::string & name) const override;
    bool getSixAxisForceTorqueSensorMeasure(std::size_t sens_index, yarp::sig::Vector & out, double & timestamp) const override;

private:
    void loadFilters(int id);

    int fd {0};
    force_array fs[4];
    unsigned long int filters[4];
    std::vector<std::string> names {"ch0", "ch1", "ch2", "ch3"};
    bool isDextrorotary {false};
};

} // namespace roboticslab

#endif // __JR3_HPP__
