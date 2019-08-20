// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "LacqueyFetch.hpp"

// -----------------------------------------------------------------------------

bool roboticslab::LacqueyFetch::send(uint32_t cob, uint16_t len, uint8_t * msgData)
{
    return sender->prepareMessage(message_builder(cob + canId, len, msgData));
}

// -----------------------------------------------------------------------------
