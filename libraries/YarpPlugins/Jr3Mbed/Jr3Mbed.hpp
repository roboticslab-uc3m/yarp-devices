// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __JR3_MBED_HPP__
#define __JR3_MBED_HPP__

#include <mutex>

#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/MultipleAnalogSensorsInterfaces.h>

#include "ICanBusSharer.hpp"

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
                public yarp::dev::ISixAxisForceTorqueSensors,
                public ICanBusSharer
{
public:
    Jr3Mbed()
    { }

    ~Jr3Mbed() override
    { close(); }

    //  --------- DeviceDriver Declarations. Implementation in DeviceDriverImpl.cpp ---------

    bool open(yarp::os::Searchable & config) override;
    bool close() override;

    //  --------- ICanBusSharer declarations. Implementation in LacqueyFetch.cpp ---------

    unsigned int getId() override;
    bool notifyMessage(const can_message & message) override;
    bool initialize() override;
    bool finalize() override;
    bool registerSender(ICanSenderDelegate * sender) override;
    bool synchronize(double timestamp) override;

    //  --------- ISixAxisForceTorqueSensors Declarations. Implementation in ISixAxisForceTorqueSensorsImpl.cpp ---------

    std::size_t getNrOfSixAxisForceTorqueSensors() const override;
    yarp::dev::MAS_status getSixAxisForceTorqueSensorStatus(std::size_t sens_index) const override;
    bool getSixAxisForceTorqueSensorName(std::size_t sens_index, std::string & name) const override;
    bool getSixAxisForceTorqueSensorFrameName(std::size_t sens_index, std::string & name) const override;
    bool getSixAxisForceTorqueSensorMeasure(std::size_t sens_index, yarp::sig::Vector & out, double & timestamp) const override;

private:
    unsigned int canId {0};
    ICanSenderDelegate * sender {nullptr};
    mutable std::mutex rxMutex;

    double fx {0.0};
    double fy {0.0};
    double fz {0.0};
    double mx {0.0};
    double my {0.0};
    double mz {0.0};
};

} // namespace roboticslab

#endif // __JR3_MBED_HPP__
