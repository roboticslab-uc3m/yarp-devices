// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __SYNAPSE_HPP__
#define __SYNAPSE_HPP__

#include <array>
#include <string>
#include <utility>

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
    typedef std::array<setpoint_t, DATA_POINTS> Setpoints;

    static const std::array<std::pair<setpoint_t, setpoint_t>, DATA_POINTS> LIMITS;
    static const std::array<std::string, DATA_POINTS> LABELS;

    Synapse();
    virtual ~Synapse() {}

    virtual void configure(void * handle);

    bool readDataList(Setpoints & setpoints);
    bool writeSetpointList(const Setpoints & setpoints);

protected:
    virtual bool getMessage(unsigned char * msg, char stopByte, int size) = 0;
    virtual bool sendMessage(unsigned char * msg, int size) = 0;

    bool configured;
};

}  // namespace roboticslab

#endif  // __SYNAPSE_HPP__
