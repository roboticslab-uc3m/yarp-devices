// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __CAN_MESSAGE_NOTIFIER_HPP__
#define __CAN_MESSAGE_NOTIFIER_HPP__

#include "CanMessage.hpp"

namespace roboticslab
{

/**
 * @ingroup CanBusSharerLib
 * @brief Implementation-agnostic consumer for TX CAN transfers.
 *
 * Implementors can use this class to forward implementation-specific CAN message
 * structures from the internal TX buffer (if any) to the point of consumption
 * and the final CAN read routines.
 *
 * Cf. @ref CanSenderDelegate.
 */
class CanMessageNotifier
{
public:
    //! Virtual destructor.
    virtual ~CanMessageNotifier() = default;

    //! Notify observers that a new CAN message has arrived.
    virtual bool notifyMessage(const can_message & msg) = 0;
};

} // namespace roboticslab

#endif // __CAN_MESSAGE_NOTIFIER_HPP__
