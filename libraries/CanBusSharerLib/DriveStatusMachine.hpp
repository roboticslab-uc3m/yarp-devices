// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __DRIVE_STATUS_MACHINE_HPP__
#define __DRIVE_STATUS_MACHINE_HPP__

#include <cstdint>

#include <bitset>
#include <mutex>

#include "PdoProtocol.hpp"
#include "StateObserver.hpp"

namespace roboticslab
{

/**
 * @ingroup CanBusSharerLib
 * @brief CiA 402 drive states (associated to statusword).
 */
enum class DriveState
{
    NOT_READY_TO_SWITCH_ON, ///< Not ready to switch on state
    SWITCH_ON_DISABLED, ///< Switch on disabled state
    READY_TO_SWITCH_ON, ///< Ready to switch on state
    SWITCHED_ON, ///< Switched on (operation disabled) state
    OPERATION_ENABLED, ///< Operation enabled state
    QUICK_STOP_ACTIVE, ///< Quick stop active state
    FAULT_REACTION_ACTIVE, ///< Fault reaction active state
    FAULT ///< Fault state
};

/**
 * @ingroup CanBusSharerLib
 * @brief CiA 402 drive transitions (associated to controlword).
 */
enum class DriveTransition : std::uint16_t
{
    SHUTDOWN = 0x0006, ///< Shutdown command (xxxx.xxxx.0xxx.x110)
    SWITCH_ON = 0x0007, ///< Switch on command (xxxx.xxxx.0xxx.0111)
    DISABLE_VOLTAGE = 0x0000, ///< Disable voltage command (xxxx.xxxx.0xxx.xx0x)
    QUICK_STOP = 0x0002, ///< Quick stop command (xxxx.xxxx.0xxx.x01x)
    ENABLE_OPERATION = 0x000F, ///< Enable operation command (xxxx.xxxx.0xxx.1111)
    FAULT_RESET = 0x0080, ///< Reset fault command (xxxx.xxxx.1xxx.xxxx)
    DISABLE_OPERATION = SWITCH_ON ///< Disable operation command, alias for DriveTransition::SWITCH_ON
};

/**
 * @ingroup CanBusSharerLib
 * @brief Representation of a CiA 402 state machine controller.
 *
 * Manages internally a bit representation for both controlword (object 6040h)
 * and statusword (object 6041h). This class hardcodes a list of allowed state
 * transitions (should cover all of CiA 402 standard). Also, it supports a subset
 * of state requests: it knows how to traverse the @ref DriveState::SWITCH_ON_DISABLED
 * to @ref DriveState::OPERATION_ENABLED chain in both directions, and
 * @ref DriveState::QUICK_STOP_ACTIVE to @ref DriveState::SWITCH_ON_DISABLED.
 * Fault resets as well as quick stop transitions must be requested individually.
 */
class DriveStatusMachine
{
public:
    typedef std::bitset<16> word_t; ///< Fixed-size sequence of 16 bits

    //! Constructor, registers RPDO handle.
    DriveStatusMachine(ReceivePdo * rpdo, double timeout)
        : rpdo(rpdo), stateObserver(timeout)
    { }

    //! Configure RPDO handle.
    void configureRpdo(ReceivePdo * rpdo)
    { this->rpdo = rpdo; }

    //! Notify observers on a drive state change, if applicable.
    bool update(std::uint16_t statusword);

    //! Retrieve stored controlword.
    word_t controlword() const;

    //! Send command via object 6040h and update stored controlword.
    bool controlword(const word_t & controlbits);

    //! Retrieve stored statusword.
    word_t statusword() const;

    //! Parse stored bit representation of a statusword into a @ref DriveState enumerator.
    DriveState getCurrentState() const;

    //! Request given drive transition via object 6040h, update stored controlword.
    bool requestTransition(DriveTransition transition, bool wait = true);

    //! Request given drive state, update stored controlword.
    bool requestState(DriveState goalState);

    //! Await until drive reaches given state, with timeout.
    bool awaitState(DriveState goalState);

    //! Parse bit representation into a @ref DriveState enumerator.
    static DriveState parseStatusword(std::uint16_t statusword);

private:
    word_t _controlword;
    word_t _statusword;
    ReceivePdo * rpdo;
    StateObserver stateObserver;
    mutable std::mutex stateMutex;
};

} // namespace roboticslab

#endif // __DRIVE_STATUS_MACHINE_HPP__
