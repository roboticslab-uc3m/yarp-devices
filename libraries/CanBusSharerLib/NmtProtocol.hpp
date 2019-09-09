// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __NMT_PROTOCOL_HPP__
#define __NMT_PROTOCOL_HPP__

#include <cstdint>

#include <functional>

#include "CanSenderDelegate.hpp"
#include "SdoClient.hpp"

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
    typedef std::function<void(NmtState)> HandlerFn;

    static constexpr std::uint8_t BROADCAST = 0;

    NmtProtocol(std::uint8_t id, SdoClient * sdo) : id(id), sdo(sdo), sender(nullptr)
    { }

    void configureSender(CanSenderDelegate * sender)
    { this->sender = sender; }

    bool issueServiceCommand(NmtService command);

    bool setupHeartbeat(std::uint16_t period);

    bool accept(const std::uint8_t * data);

    void registerHandler(const HandlerFn & fn)
    { callback = fn; }

private:
    std::uint8_t id;
    SdoClient * sdo;
    CanSenderDelegate * sender;

    HandlerFn callback;
};

} // namespace roboticslab

#endif // __NMT_PROTOCOL_HPP__
