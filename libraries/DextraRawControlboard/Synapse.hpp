// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __SYNAPSE_HPP__
#define __SYNAPSE_HPP__

#include <utility>

#include <yarp/dev/SerialInterfaces.h>

namespace roboticslab
{

/**
 * @ingroup DextraRawControlboard
 * @brief Comms layer to interface with the onboard Arduino through serial port.
 * C++ port of <a href="https://github.com/Alvipe/Dextra/blob/30c7524/Control/synapse.py">synapse.py</a>.
 */
class Synapse
{
public:
    static const int DATA_POINTS = 6;

    typedef float setpoint_t;
    typedef setpoint_t Setpoints[DATA_POINTS];

    static const std::pair<setpoint_t, setpoint_t> LIMITS[DATA_POINTS];
    static const char * LABELS[DATA_POINTS];

    virtual ~Synapse() {}

    bool readDataList(Setpoints & setpoints);
    bool writeSetpointList(const Setpoints & setpoints);

protected:
    virtual bool getMessage(unsigned char * msg, char stopByte, int size) = 0;
    virtual bool sendMessage(char * msg, int size) = 0;
};

}  // namespace roboticslab

#endif  // __SYNAPSE_HPP__
