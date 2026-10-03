// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __JR3_PCI_HPP__
#define __JR3_PCI_HPP__

#include <yarp/conf/version.h>

#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/MultipleAnalogSensorsInterfaces.h>

#include <jr3pci-ioctl.h>

#include "Jr3Pci_ParamsParser.h"

/**
 * @ingroup YarpPlugins
 * @defgroup Jr3Pci
 * @brief Contains Jr3Pci.
 */

 /**
 * @ingroup Jr3Pci
 * @brief Implementation for the JR3 sensor (PCi board).
 */
class Jr3Pci : public yarp::dev::DeviceDriver,
               public yarp::dev::ISixAxisForceTorqueSensors,
               public Jr3Pci_ParamsParser
{
public:
    //  --------- DeviceDriver Declarations. Implementation in DeviceDriverImpl.cpp ---------
    bool open(yarp::os::Searchable& config) override;
    bool close() override;

    //  --------- ISixAxisForceTorqueSensors Declarations. Implementation in ISixAxisForceTorqueSensorsImpl.cpp ---------
#if YARP_VERSION_COMPARE(>=, 4, 1, 0) || defined(YARP_NEXT)
    yarp::dev::ReturnValue getNrOfSixAxisForceTorqueSensors(std::size_t & num) const override;
    yarp::dev::MAS_status getSixAxisForceTorqueSensorStatus(std::size_t sens_index) const override;
    yarp::dev::ReturnValue getSixAxisForceTorqueSensorName(std::size_t sens_index, std::string & name) const override;
    yarp::dev::ReturnValue getSixAxisForceTorqueSensorFrameName(std::size_t sens_index, std::string & frameName) const override;
    yarp::dev::ReturnValue getSixAxisForceTorqueSensorMeasure(std::size_t sens_index, yarp::sig::Vector & out, double & timestamp) const override;
#else
    std::size_t getNrOfSixAxisForceTorqueSensors() const override;
    yarp::dev::MAS_status getSixAxisForceTorqueSensorStatus(std::size_t sens_index) const override;
    bool getSixAxisForceTorqueSensorName(std::size_t sens_index, std::string & name) const override;
    bool getSixAxisForceTorqueSensorFrameName(std::size_t sens_index, std::string & frameName) const override;
    bool getSixAxisForceTorqueSensorMeasure(std::size_t sens_index, yarp::sig::Vector & out, double & timestamp) const override;
#endif

private:
    void loadFilters(int id);
    bool calibrateSensor();
    bool calibrateChannel(int ch);

    int fd {0};
    force_array fs[4];
    unsigned long int filters[4];
};

#endif // __JR3_PCI_HPP__
