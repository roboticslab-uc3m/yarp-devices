// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusFake.hpp"

// -----------------------------------------------------------------------------

bool roboticslab::CanBusFake::sendRaw(uint32_t id, uint16_t len, uint8_t * msgData)
{
    return true;
}

// -----------------------------------------------------------------------------

int roboticslab::CanBusFake::read_timeout(struct can_msg *buf, unsigned int timeout)
{
    return 0;  //-- If gets to here, return whatever read() returned.
}

// -----------------------------------------------------------------------------

