// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "NmtProtocol.hpp"

using namespace roboticslab;

bool NmtProtocol::issueServiceCommand(NmtService command)
{
    std::uint8_t msg[] = {static_cast<std::uint8_t>(command), id};
    return sender && sender->prepareMessage({0, 2, msg});
}

bool NmtProtocol::accept(const std::uint8_t * data)
{
    if (!callback)
    {
        return false;
    }

    NmtState state;

    switch (data[0])
    {
    case 0:
        state = NmtState::BOOTUP;
        break;
    case 4:
        state = NmtState::STOPPED;
        break;
    case 5:
        state = NmtState::OPERATIONAL;
        break;
    case 127:
        state = NmtState::PRE_OPERATIONAL;
        break;
    default:
        return false;
    }

    callback(state);
    return true;
}
