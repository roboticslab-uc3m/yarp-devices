// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __SYNAPSE_HPP__
#define __SYNAPSE_HPP__

#include <yarp/conf/numeric.h>

#include <utility>

namespace roboticslab
{

/**
 * @ingroup DextraRawControlboard
 * @brief Base comms layer to interface with the Dextra's Arduino board.
 *
 * C++ port of <a href="https://github.com/Alvipe/Dextra/blob/30c7524/Control/synapse.py">synapse.py</a>.
 */
class Synapse
{
public:
    static const int DATA_POINTS = 6; ///< Number of controlled motors

    typedef yarp::conf::float32_t setpoint_t; ///< 4-byte long float
    typedef setpoint_t Setpoints[DATA_POINTS]; ///< Container of Dextra setpoints.

    static const std::pair<setpoint_t, setpoint_t> LIMITS[DATA_POINTS]; ///< Joint limits per motor
    static const char * LABELS[DATA_POINTS]; ///< String labels that identify each motor

    //! Virtual destructor.
    virtual ~Synapse() {}

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
