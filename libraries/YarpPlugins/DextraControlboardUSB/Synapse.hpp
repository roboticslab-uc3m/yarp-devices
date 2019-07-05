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

    static const int DATA_POINTS = 6;

    typedef float setpoint_t;
    typedef setpoint_t Setpoints[DATA_POINTS];

    Synapse(yarp::dev::ISerialDevice * iSerialDevice);

    bool readDataList(Setpoints & setpoints);

    bool writeSetpointList(const Setpoints & setpoints);

private:

    bool getMessage(unsigned char * msg);

    yarp::dev::ISerialDevice * iSerialDevice;
};

}  // namespace roboticslab

#endif  // __SYNAPSE_HPP__
