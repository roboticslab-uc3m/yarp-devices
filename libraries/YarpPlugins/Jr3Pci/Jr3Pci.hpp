// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __JR3_PCI_HPP__
#define __JR3_PCI_HPP__

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
    std::size_t getNrOfSixAxisForceTorqueSensors() const override;
    yarp::dev::MAS_status getSixAxisForceTorqueSensorStatus(std::size_t sens_index) const override;
    bool getSixAxisForceTorqueSensorName(std::size_t sens_index, std::string & name) const override;
    bool getSixAxisForceTorqueSensorFrameName(std::size_t sens_index, std::string & name) const override;
    bool getSixAxisForceTorqueSensorMeasure(std::size_t sens_index, yarp::sig::Vector & out, double & timestamp) const override;

private:
    void loadFilters(int id);
    bool calibrateSensor();
    bool calibrateChannel(int ch);

    int fd {0};
    force_array fs[4];
    unsigned long int filters[4];
};

#endif // __JR3_PCI_HPP__
