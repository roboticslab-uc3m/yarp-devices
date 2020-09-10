// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __I_CAN_BUS_SHARER_HPP__
#define __I_CAN_BUS_SHARER_HPP__

#include <vector>

#include "CanMessageNotifier.hpp"
#include "CanSenderDelegate.hpp"

namespace roboticslab
{

/**
 * @ingroup yarp_devices_libraries
 * @defgroup CanBusSharerLib
 * @brief Common interfaces and utilities for a network of CAN nodes.
 */

/**
 * @ingroup CanBusSharerLib
 * @brief Abstract base for a CAN bus sharer.
 */
class ICanBusSharer : public CanMessageNotifier
{
public:
    //! Destructor.
    virtual ~ICanBusSharer() = default;

    //! Retrieve CAN node ID.
    virtual unsigned int getId() = 0;

    //! Retrieve more associated CAN node IDs, if any.
    virtual std::vector<unsigned int> getAdditionalIds()
    { return {}; }

    //! Perform CAN node initialization.
    virtual bool initialize() = 0;

    //! Finalize CAN node communications.
    virtual bool finalize() = 0;

    //! Pass a handle to a CAN sender delegate instance.
    virtual bool registerSender(CanSenderDelegate * sender) = 0;

    //! Perform synchronized action on CAN master's request.
    virtual bool synchronize() = 0;
};

} // namespace roboticslab

#endif // __I_CAN_BUS_SHARER_HPP__
