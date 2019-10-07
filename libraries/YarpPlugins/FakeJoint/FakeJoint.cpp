// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "FakeJoint.hpp"

// -----------------------------------------------------------------------------

bool roboticslab::FakeJoint::send(uint32_t cob, uint16_t len, uint8_t * msgData)
{
    return sender->prepareMessage({cob + canId, len, msgData});
}

// -----------------------------------------------------------------------------
