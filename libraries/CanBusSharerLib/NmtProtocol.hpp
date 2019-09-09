// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __NMT_PROTOCOL_HPP__
#define __NMT_PROTOCOL_HPP__

#include <cstdint>

#include "CanSenderDelegate.hpp"

namespace roboticslab
{

enum class NmtService
{
    START_REMOTE_NODE = 1,
    STOP_REMOTE_NODE = 2,
    ENTER_PRE_OPERATIONAL = 128,
    RESET_NODE = 129,
    RESET_COMMUNICATION = 130
};

enum class NmtState
{
    BOOTUP = 0,
    STOPPED = 4,
    OPERATIONAL = 5,
    PRE_OPERATIONAL = 127
};

class NmtProtocol final
{
public:
    static constexpr std::uint8_t BROADCAST = 0;

    NmtProtocol(std::uint8_t id) : id(id), sender(nullptr)
    { }

    bool issueServiceCommand(NmtService command);

    void configureSender(CanSenderDelegate * sender)
    { this->sender = sender; }

private:
    std::uint8_t id;
    CanSenderDelegate * sender;
};

} // namespace roboticslab

#endif // __NMT_PROTOCOL_HPP__
