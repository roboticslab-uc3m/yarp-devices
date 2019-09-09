// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "NmtProtocol.hpp"

using namespace roboticslab;

bool NmtProtocol::issueServiceCommand(NmtService command)
{
    std::uint8_t msg[] = {static_cast<std::uint8_t>(command), id};
    return sender->prepareMessage(message_builder(0, 2, msg));
}
