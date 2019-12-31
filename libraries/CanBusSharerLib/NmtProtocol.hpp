// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __NMT_PROTOCOL_HPP__
#define __NMT_PROTOCOL_HPP__

#include <cstdint>

#include <functional>

#include "CanSenderDelegate.hpp"
#include "SdoClient.hpp"

namespace roboticslab
{

/**
 * @ingroup CanBusSharerLib
 * @brief NMT service commands.
 */
enum class NmtService : std::uint8_t
{
    START_REMOTE_NODE = 1, ///< Start remote node indication
    STOP_REMOTE_NODE = 2, ///< Stop remote node indication
    ENTER_PRE_OPERATIONAL = 128, ///< Enter pre-operational indication
    RESET_NODE = 129, ///< Reset node indication
    RESET_COMMUNICATION = 130 ///< Reset communication indication
};

/**
 * @ingroup CanBusSharerLib
 * @brief NMT state machine.
 */
enum class NmtState : std::uint8_t
{
    BOOTUP = 0, ///< Initial bootup
    STOPPED = 4, ///< Stopped state
    OPERATIONAL = 5, ///< Operational state
    PRE_OPERATIONAL = 127 ///< Pre-operational state
};

/**
 * @ingroup CanBusSharerLib
 * @brief Representation of NMT protocol.
 */
class NmtProtocol final
{
public:
    static constexpr std::uint8_t BROADCAST = 0; ///< Broadcast CAN ID

    //! Constructor, registers SDO client and CAN sender handles.
    NmtProtocol(std::uint8_t id, SdoClient * sdo, CanSenderDelegate * sender = nullptr)
        : id(id), sdo(sdo), sender(sender)
    { }

    //! Configure CAN sender delegate handle.
    void configureSender(CanSenderDelegate * sender)
    { this->sender = sender; }

    //! Send NMT service indication.
    bool issueServiceCommand(NmtService command);

    //! Configure heartbeat-related CAN objects via SDO.
    bool setupHeartbeat(std::uint16_t period);

    //! Invoke callback on parsed CAN message data.
    bool accept(const std::uint8_t * data);

    //! Register callback.
    template<typename Fn>
    void registerHandler(Fn && fn)
    { callback = fn; }

    //! Unregister callback.
    void unregisterHandler()
    { callback = HandlerFn(); }

private:
    typedef std::function<void(NmtState)> HandlerFn;

    std::uint8_t id;
    SdoClient * sdo;
    CanSenderDelegate * sender;

    HandlerFn callback;
};

} // namespace roboticslab

#endif // __NMT_PROTOCOL_HPP__
