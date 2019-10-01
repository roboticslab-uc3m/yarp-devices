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

// associated to statusword
enum class DriveState
{
    NOT_READY_TO_SWITCH_ON,
    SWITCH_ON_DISABLED,
    READY_TO_SWITCH_ON,
    SWITCHED_ON,
    OPERATION_ENABLED,
    QUICK_STOP_ACTIVE,
    FAULT_REACTION_ACTIVE,
    FAULT
};

// associated to controlword
enum class DriveTransition
{
    SHUTDOWN,
    SWITCH_ON,
    DISABLE_VOLTAGE,
    QUICK_STOP,
    ENABLE_OPERATION,
    DISABLE_OPERATION, // same as switch on
    FAULT_RESET
};

class DriveStatusMachine
{
public:
    typedef std::bitset<16> word_t;

    DriveStatusMachine(ReceivePdo * rpdo, double timeout)
        : rpdo(rpdo), stateObserver(timeout)
    { }

    void configureRpdo(ReceivePdo * rpdo)
    { this->rpdo = rpdo; }

    bool update(std::uint16_t statusword);
    word_t & controlword();
    const word_t & statusword() const;
    DriveState getCurrentState() const;
    bool requestTransition(DriveTransition transition, bool wait = true);
    bool requestState(DriveState goalState);

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
