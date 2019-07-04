// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __SYNAPSE_HPP__
#define __SYNAPSE_HPP__

#include <vector>

#include <yarp/dev/SerialInterfaces.h>

namespace roboticslab
{

/**
* @ingroup DextraControlboardUSB
* @brief Comms layer to interface with the onboard Arduino through serial port.
*/
class Synapse
{

public:

    Synapse();

    void setSerialDeviceHandle(yarp::dev::ISerialDevice * iSerialDevice)
    { this->iSerialDevice = iSerialDevice; }

    bool readDataList(std::vector<double> & setpoints);

    bool writeSetpointList(const std::vector<double> & setpoints);

    static const int DATA_POINTS = 6;

private:

    bool getMessage(unsigned char * msg);

    yarp::dev::ISerialDevice * iSerialDevice;
};

}  // namespace roboticslab

#endif  // __SYNAPSE_HPP__
