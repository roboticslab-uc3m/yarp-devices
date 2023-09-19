// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __JR3_MBED_HPP__
#define __JR3_MBED_HPP__

#include <vector>

#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/MultipleAnalogSensorsInterfaces.h>
#include <yarp/dev/PolyDriver.h>

#include "CanReadThread.hpp"

namespace roboticslab
{

/**
 * @ingroup YarpPlugins
 * @defgroup Jr3Mbed
 * @brief Contains roboticslab::Jr3Mbed.
 */

 /**
 * @ingroup Jr3Mbed
 * @brief Implementation of a CAN node on an Mbed board that publishes data from a JR3 sensor.
 */
class Jr3Mbed : public yarp::dev::DeviceDriver,
                public yarp::dev::ISixAxisForceTorqueSensors
{
public:
    Jr3Mbed()
    { }

    ~Jr3Mbed() override
    { close(); }

    //  --------- DeviceDriver Declarations. Implementation in DeviceDriverImpl.cpp ---------
    bool open(yarp::os::Searchable & config) override;
    bool close() override;

    //  --------- ISixAxisForceTorqueSensors Declarations. Implementation in ISixAxisForceTorqueSensorsImpl.cpp ---------
    std::size_t getNrOfSixAxisForceTorqueSensors() const override;
    yarp::dev::MAS_status getSixAxisForceTorqueSensorStatus(std::size_t sens_index) const override;
    bool getSixAxisForceTorqueSensorName(std::size_t sens_index, std::string & name) const override;
    bool getSixAxisForceTorqueSensorFrameName(std::size_t sens_index, std::string & name) const override;
    bool getSixAxisForceTorqueSensorMeasure(std::size_t sens_index, yarp::sig::Vector & out, double & timestamp) const override;

private:
    std::vector<yarp::dev::PolyDriver *> canBusDevices;
    std::vector<CanReadThread *> canReadThreads;
};

} // namespace roboticslab

#endif // __JR3_MBED_HPP__
