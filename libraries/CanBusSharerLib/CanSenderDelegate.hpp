// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __CAN_SENDER_DELEGATE_HPP__
#define __CAN_SENDER_DELEGATE_HPP__

#include "CanMessage.hpp"

namespace roboticslab
{

/**
 * @ingroup CanBusSharerLib
 * @brief Implementation-agnostic consumer for RX CAN transfers.
 *
 * Implementors can use this class to forward implementation-specific CAN message
 * structures from the point of creation down to the internal RX buffer (if any)
 * and the final CAN write routines.
 *
 * Cf. @ref CanMessageNotifier.
 */
class CanSenderDelegate
{
public:
    //! Virtual destructor.
    virtual ~CanSenderDelegate() = default;

    //! Register CAN message for write.
    virtual bool prepareMessage(const can_message & msg) = 0;
};

} // namespace roboticslab

#endif // __CAN_SENDER_DELEGATE_HPP__
