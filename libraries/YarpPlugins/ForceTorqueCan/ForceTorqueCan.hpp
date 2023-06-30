// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __FORCE_TORQUE_CAN_HPP__
#define __FORCE_TORQUE_CAN_HPP__

#include <yarp/os/Thread.h>

#include <yarp/dev/CanBusInterface.h>
#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/MultipleAnalogSensorsInterfaces.h>
#include <yarp/dev/PolyDriver.h>

namespace roboticslab
{

/**
 * @ingroup YarpPlugins
 * @defgroup ForceTorqueCan
 * @brief Contains roboticslab::ForceTorqueCan.
 */

 /**
 * @ingroup ForceTorqueCan
 * @brief Implementation of a CAN node that publishes data from a force-torque sensor.
 */
class ForceTorqueCan : public yarp::dev::DeviceDriver,
                       public yarp::dev::ISixAxisForceTorqueSensors,
                       public yarp::os::Thread
{
public:
    ForceTorqueCan()
    { }

    ~ForceTorqueCan() override
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

    //  --------- Thread Declarations. Implementation in ThreadImpl.cpp ---------
    bool threadInit() override;
    void threadRelease() override;
    void run() override;

private:
    unsigned int bufferSize {0};
    yarp::dev::PolyDriver canBus;

    yarp::dev::ICanBus * iCanBus {nullptr};
    yarp::dev::ICanBufferFactory * iCanBufferFactory {nullptr};
    yarp::dev::CanBuffer canBuffer;
};

} // namespace roboticslab

#endif // __FORCE_TORQUE_CAN_HPP__
