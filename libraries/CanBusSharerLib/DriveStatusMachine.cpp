// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "DriveStatusMachine.hpp"

#include <bitset>
#include <unordered_map>
#include <vector>

using namespace roboticslab;

namespace
{
    DriveState parseDriveState(const std::bitset<16> & bits)
    {
        if (!bits.test(0) && !bits.test(1) && !bits.test(2) && !bits.test(3) && !bits.test(6))
        {
            return DriveState::NOT_READY_TO_SWITCH_ON; // xxxx.xxxx.x0xx.0000
        }
        else if (!bits.test(0) && !bits.test(1) && !bits.test(2) && !bits.test(3) && bits.test(6))
        {
            return DriveState::SWITCH_ON_DISABLED; // xxxx.xxxx.x1xx.0000
        }
        else if (bits.test(0) && !bits.test(1) && !bits.test(2) && !bits.test(3) && bits.test(5) && !bits.test(6))
        {
            return DriveState::READY_TO_SWITCH_ON; // xxxx.xxxx.x01x.0001
        }
        else if (bits.test(0) && bits.test(1) && !bits.test(2) && !bits.test(3) && bits.test(5) && !bits.test(6))
        {
            return DriveState::SWITCHED_ON; // xxxx.xxxx.x01x.0011
        }
        else if (bits.test(0) && bits.test(1) && bits.test(2) && !bits.test(3) && bits.test(5) && !bits.test(6))
        {
            return DriveState::OPERATION_ENABLED; // xxxx.xxxx.x01x.0111
        }
        else if (bits.test(0) && bits.test(1) && bits.test(2) && !bits.test(3) && !bits.test(5) && !bits.test(6))
        {
            return DriveState::QUICK_STOP_ACTIVE; // xxxx.xxxx.x00x.0111
        }
        else if (bits.test(0) && bits.test(1) && bits.test(2) && bits.test(3) && !bits.test(6))
        {
            return DriveState::FAULT_REACTION_ACTIVE; // xxxx.xxxx.x0xx.1111
        }
        else if (!bits.test(0) && !bits.test(1) && !bits.test(2) && bits.test(3) && !bits.test(6))
        {
            return DriveState::FAULT; // xxxx.xxxx.x0xx.1000
        }
        else
        {
            return DriveState::NOT_READY_TO_SWITCH_ON; // assume drive is dead
        }
    }

    void prepareDriveTransition(DriveTransition transition, std::bitset<16> & bits)
    {
        switch (transition)
        {
        case DriveTransition::SHUTDOWN: // xxxx.xxxx.0xxx.x110
            bits.reset(0);
            bits.set(1);
            bits.set(2);
            bits.reset(7);
            break;
        case DriveTransition::SWITCH_ON: // xxxx.xxxx.0xxx.0111
        case DriveTransition::DISABLE_OPERATION:
            bits.set(0);
            bits.set(1);
            bits.set(2);
            bits.reset(3);
            bits.reset(7);
            break;
        case DriveTransition::DISABLE_VOLTAGE: // xxxx.xxxx.0xxx.xx0x
            bits.reset(1);
            bits.reset(7);
            break;
        case DriveTransition::QUICK_STOP: // xxxx.xxxx.0xxx.x01x
            bits.set(1);
            bits.reset(2);
            bits.reset(7);
            break;
        case DriveTransition::ENABLE_OPERATION: // xxxx.xxxx.0xxx.1111
            bits.set(0);
            bits.set(1);
            bits.set(2);
            bits.set(3);
            bits.reset(7);
            break;
        case DriveTransition::FAULT_RESET: // xxxx.xxxx.^xxx.xxxx
            bits.set(7); // FIXME: check this
            break;
        }
    }

    const std::unordered_map<std::pair<DriveState, DriveTransition>, DriveState> transitionToState = {
        {std::make_pair(DriveState::SWITCH_ON_DISABLED, DriveTransition::SHUTDOWN), DriveState::READY_TO_SWITCH_ON}, // 2
        {std::make_pair(DriveState::READY_TO_SWITCH_ON, DriveTransition::SWITCH_ON), DriveState::SWITCHED_ON}, // 3
        {std::make_pair(DriveState::SWITCHED_ON, DriveTransition::ENABLE_OPERATION), DriveState::OPERATION_ENABLED}, // 4
        {std::make_pair(DriveState::OPERATION_ENABLED, DriveTransition::DISABLE_OPERATION), DriveState::SWITCHED_ON}, // 5
        {std::make_pair(DriveState::OPERATION_ENABLED, DriveTransition::SWITCH_ON), DriveState::SWITCHED_ON}, // 5 (alias)
        {std::make_pair(DriveState::SWITCHED_ON, DriveTransition::SHUTDOWN), DriveState::READY_TO_SWITCH_ON}, // 6
        {std::make_pair(DriveState::READY_TO_SWITCH_ON, DriveTransition::DISABLE_VOLTAGE), DriveState::SWITCH_ON_DISABLED}, // 7
        {std::make_pair(DriveState::OPERATION_ENABLED, DriveTransition::SHUTDOWN), DriveState::READY_TO_SWITCH_ON}, // 8
        {std::make_pair(DriveState::OPERATION_ENABLED, DriveTransition::DISABLE_VOLTAGE), DriveState::SWITCH_ON_DISABLED}, // 9
        {std::make_pair(DriveState::SWITCHED_ON, DriveTransition::DISABLE_VOLTAGE), DriveState::SWITCH_ON_DISABLED}, // 10
        {std::make_pair(DriveState::OPERATION_ENABLED, DriveTransition::QUICK_STOP), DriveState::QUICK_STOP_ACTIVE}, // 11
        {std::make_pair(DriveState::QUICK_STOP_ACTIVE, DriveTransition::DISABLE_VOLTAGE), DriveState::SWITCH_ON_DISABLED}, // 12
        {std::make_pair(DriveState::QUICK_STOP_ACTIVE, DriveTransition::ENABLE_OPERATION), DriveState::OPERATION_ENABLED}, // 16
    };

    const std::unordered_map<std::pair<DriveState, DriveState>, std::vector<DriveTransition>> shortestPaths = {
        {std::make_pair(DriveState::SWITCH_ON_DISABLED, DriveState::READY_TO_SWITCH_ON), {DriveTransition::SHUTDOWN}},
        {std::make_pair(DriveState::SWITCH_ON_DISABLED, DriveState::SWITCHED_ON), {DriveTransition::SHUTDOWN, DriveTransition::SWITCH_ON}},
        {std::make_pair(DriveState::SWITCH_ON_DISABLED, DriveState::OPERATION_ENABLED), {DriveTransition::SHUTDOWN, DriveTransition::SWITCH_ON, DriveTransition::ENABLE_OPERATION}},
        {std::make_pair(DriveState::READY_TO_SWITCH_ON, DriveState::SWITCHED_ON), {DriveTransition::SWITCH_ON}},
        {std::make_pair(DriveState::READY_TO_SWITCH_ON, DriveState::OPERATION_ENABLED), {DriveTransition::SWITCH_ON, DriveTransition::ENABLE_OPERATION}},
        {std::make_pair(DriveState::SWITCHED_ON, DriveState::OPERATION_ENABLED), {DriveTransition::ENABLE_OPERATION}},
        {std::make_pair(DriveState::OPERATION_ENABLED, DriveState::SWITCHED_ON), {DriveTransition::DISABLE_OPERATION}},
        {std::make_pair(DriveState::OPERATION_ENABLED, DriveState::READY_TO_SWITCH_ON), {DriveTransition::SHUTDOWN}},
        {std::make_pair(DriveState::OPERATION_ENABLED, DriveState::SWITCH_ON_DISABLED), {DriveTransition::DISABLE_VOLTAGE}},
        {std::make_pair(DriveState::SWITCHED_ON, DriveState::READY_TO_SWITCH_ON), {DriveTransition::SHUTDOWN}},
        {std::make_pair(DriveState::SWITCHED_ON, DriveState::SWITCH_ON_DISABLED), {DriveTransition::DISABLE_VOLTAGE}},
        {std::make_pair(DriveState::READY_TO_SWITCH_ON, DriveState::SWITCH_ON_DISABLED), {DriveTransition::DISABLE_VOLTAGE}},
        {std::make_pair(DriveState::READY_TO_SWITCH_ON, DriveState::SWITCHED_ON), {DriveTransition::SWITCH_ON}},
        {std::make_pair(DriveState::QUICK_STOP_ACTIVE, DriveState::SWITCH_ON_DISABLED), {DriveTransition::DISABLE_VOLTAGE}}
    };
}

bool DriveStatusMachine::update(std::uint16_t statusword)
{
    {
        std::lock_guard<std::mutex> lock(stateMutex);
        _statusword = statusword;
    }

    return stateObserver.notify();
}

DriveStatusMachine::word_t & DriveStatusMachine::controlword()
{
    return _controlword;
}

const DriveStatusMachine::word_t & DriveStatusMachine::statusword() const
{
    std::lock_guard<std::mutex> lock(stateMutex);
    return _statusword;
}

DriveState DriveStatusMachine::getCurrentState() const
{
    return parseDriveState(statusword());
}

bool DriveStatusMachine::requestTransition(DriveTransition transition, bool wait)
{
    DriveState initialState = getCurrentState();
    auto op = std::make_pair(initialState, transition);
    auto it = transitionToState.find(op);

    if (it == transitionToState.cend())
    {
        return false;
    }

    prepareDriveTransition(transition, _controlword);
    std::uint16_t data = _controlword.to_ulong();

    if (!rpdo->write(data))
    {
        return false;
    }

    if (wait)
    {
        return stateObserver.await() && it->second == getCurrentState();
    }

    return true;
}

bool DriveStatusMachine::requestState(DriveState goalState, bool wait)
{
    DriveState initialState = getCurrentState();

    if (goalState == initialState)
    {
        return true;
    }

    auto op = std::make_pair(initialState, goalState);
    auto it = shortestPaths.find(op);

    if (it == shortestPaths.cend())
    {
        return false;
    }

    for (const auto & transition : it->second)
    {
        if (!requestTransition(transition, wait))
        {
            return false;
        }
    }

    return true;
}
