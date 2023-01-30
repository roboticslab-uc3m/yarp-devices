// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __I_CAN_MESSAGE_NOTIFIER_HPP__
#define __I_CAN_MESSAGE_NOTIFIER_HPP__

#include "CanMessage.hpp"

namespace roboticslab
{

/**
 * @ingroup CanBusSharerLib
 * @brief Implementation-agnostic consumer for RX CAN transfers.
 *
 * Implementors can use this class to forward implementation-specific CAN message
 * structures from the internal RX buffer (if any) to the point of consumption
 * and the final CAN read routines.
 *
 * Cf. @ref ICanSenderDelegate.
 */
class ICanMessageNotifier
{
public:
    //! Virtual destructor.
    virtual ~ICanMessageNotifier() = default;

    //! Notify observers that a new CAN message has arrived.
    virtual bool notifyMessage(const can_message & msg) = 0;
};

} // namespace roboticslab

#endif // __I_CAN_MESSAGE_NOTIFIER_HPP__
