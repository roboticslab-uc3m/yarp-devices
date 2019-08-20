// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CuiAbsolute.hpp"

// -----------------------------------------------------------------------------

/*
 * Write message to the CAN buffer.
 * @param cob Message's COB
 * @param len Data field length
 * @param msgData Data to send
 * @return true/false on success/failure.
*/
bool roboticslab::CuiAbsolute::send(uint32_t cob, uint16_t len, uint8_t * msgData)
{
    return sender->prepareMessage(message_builder(cob + canId, len, msgData));
}
