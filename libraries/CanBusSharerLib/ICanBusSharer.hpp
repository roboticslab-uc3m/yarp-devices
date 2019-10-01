// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __I_CAN_BUS_SHARER_HPP__
#define __I_CAN_BUS_SHARER_HPP__

#include <vector>

#include <yarp/dev/CanBusInterface.h>

#include "CanSenderDelegate.hpp"

namespace roboticslab
{

/**
 * @brief Abstract base for a CAN bus sharer.
 */
class ICanBusSharer
{
public:

    virtual ~ICanBusSharer() {}

    virtual unsigned int getId() = 0;

    virtual std::vector<unsigned int> getAdditionalIds()
    { return {}; }

    virtual bool initialize() = 0;

    virtual bool finalize() = 0;

    virtual bool interpretMessage(const yarp::dev::CanMessage & message) = 0;

    virtual bool registerSender(CanSenderDelegate * sender) = 0;
};

} // namespace roboticslab

#endif // __I_CAN_BUS_SHARER_HPP__
