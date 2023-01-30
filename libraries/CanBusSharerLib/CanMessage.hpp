// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __CAN_MESSAGE_HPP__
#define __CAN_MESSAGE_HPP__

namespace roboticslab
{

/**
 * @ingroup CanBusSharerLib
 * @brief Proxy CAN message structure.
 *
 * Note the data field points at externally stored bytes, therefore this
 * structure is a mere vehicle to pass CAN messages around without the
 * cost of copying too much stuff.
 *
 * See companion classes @ref ICanSenderDelegate and @ref ICanMessageNotifier.
 */
struct can_message
{
    unsigned int id;
    unsigned int len;
    const unsigned char * data;
};

} // namespace roboticslab

#endif // __CAN_MESSAGE_HPP__
