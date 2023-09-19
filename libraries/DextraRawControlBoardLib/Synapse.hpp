// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __SYNAPSE_HPP__
#define __SYNAPSE_HPP__

#include <yarp/conf/numeric.h>

#include <array>
#include <string>
#include <utility>

namespace roboticslab
{

/**
 * @ingroup DextraRawControlBoard
 * @brief Base comms layer to interface with the Dextra's Arduino board.
 *
 * C++ port of <a href="https://github.com/Alvipe/Dextra/blob/30c7524/Control/synapse.py">synapse.py</a>.
 */
class Synapse
{
public:
    static constexpr int DATA_POINTS = 6; ///< Number of controlled motors

    using setpoint_t = float; ///< 4-byte long float
    using Setpoints = std::array<setpoint_t, DATA_POINTS>; ///< Container of Dextra setpoints.

    static const std::array<std::pair<setpoint_t, setpoint_t>, DATA_POINTS> LIMITS; ///< Joint limits per motor
    static const std::array<std::string, DATA_POINTS> LABELS; ///< String labels that identify each motor

    //! Virtual destructor.
    virtual ~Synapse() = default;

    //! Configure handle for the comms interface, if necessary.
    virtual void configure(void * handle);

    //! Request current setpoints from the board.
    bool readDataList(Setpoints & setpoints);

    //! Write new setpoints to the board.
    bool writeSetpointList(const Setpoints & setpoints);

protected:
    //! Retrieve single message via comms interface.
    virtual bool getMessage(unsigned char * msg, char stopByte, int size) = 0;

    //! Forward generated message via comms interface.
    virtual bool sendMessage(unsigned char * msg, int size) = 0;

    bool configured = false;
};

}  // namespace roboticslab

#endif  // __SYNAPSE_HPP__
