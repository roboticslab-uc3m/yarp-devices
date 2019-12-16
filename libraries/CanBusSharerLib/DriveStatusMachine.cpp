// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "DriveStatusMachine.hpp"

#include <bitset>
#include <functional>
#include <unordered_map>
#include <vector>

using namespace roboticslab;

namespace std
{
    template<>
    struct hash<roboticslab::DriveState>
    {
        std::size_t operator()(roboticslab::DriveState state)
        { return static_cast<std::size_t>(state); }
    };

    template<>
    struct hash<roboticslab::DriveTransition>
    {
        std::size_t operator()(roboticslab::DriveTransition transition)
        { return static_cast<std::size_t>(transition); }
    };
}

namespace
{
    // https://www.techiedelight.com/use-std-pair-key-std-unordered_map-cpp
    struct pair_hash
    {
        template<typename T1, typename T2>
        std::size_t operator()(const std::pair<T1, T2> & pair) const
        { return std::hash<T1>()(pair.first) ^ std::hash<T2>()(pair.second); }
    };

    DriveState parseDriveState(const std::bitset<16> & bits)
    {
        if (!bits[0] && !bits[1] && !bits[2] && !bits[3] && !bits[6])
        {
            return DriveState::NOT_READY_TO_SWITCH_ON; // xxxx.xxxx.x0xx.0000
        }
        else if (!bits[0] && !bits[1] && !bits[2] && !bits[3] && bits[6])
        {
            return DriveState::SWITCH_ON_DISABLED; // xxxx.xxxx.x1xx.0000
        }
        else if (bits[0] && !bits[1] && !bits[2] && !bits[3] && bits[5] && !bits[6])
        {
            return DriveState::READY_TO_SWITCH_ON; // xxxx.xxxx.x01x.0001
        }
        else if (bits[0] && bits[1] && !bits[2] && !bits[3] && bits[5] && !bits[6])
        {
            return DriveState::SWITCHED_ON; // xxxx.xxxx.x01x.0011
        }
        else if (bits[0] && bits[1] && bits[2] && !bits[3] && bits[5] && !bits[6])
        {
            return DriveState::OPERATION_ENABLED; // xxxx.xxxx.x01x.0111
        }
        else if (bits[0] && bits[1] && bits[2] && !bits[3] && !bits[5] && !bits[6])
        {
            return DriveState::QUICK_STOP_ACTIVE; // xxxx.xxxx.x00x.0111
        }
        else if (bits[0] && bits[1] && bits[2] && bits[3] && !bits[6])
        {
            return DriveState::FAULT_REACTION_ACTIVE; // xxxx.xxxx.x0xx.1111
        }
        else if (!bits[0] && !bits[1] && !bits[2] && bits[3] && !bits[6])
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
            bits.reset(0).set(1).set(2).reset(7);
            break;
        case DriveTransition::SWITCH_ON: // xxxx.xxxx.0xxx.0111
        //case DriveTransition::DISABLE_OPERATION:
            bits.set(0).set(1).set(2).reset(3).reset(7);
            break;
        case DriveTransition::DISABLE_VOLTAGE: // xxxx.xxxx.0xxx.xx0x
            bits.reset(1).reset(7);
            break;
        case DriveTransition::QUICK_STOP: // xxxx.xxxx.0xxx.x01x
            bits.set(1).reset(2).reset(7);
            break;
        case DriveTransition::ENABLE_OPERATION: // xxxx.xxxx.0xxx.1111
            bits.set(0).set(1).set(2).set(3).reset(7);
            break;
        case DriveTransition::FAULT_RESET: // xxxx.xxxx.^xxx.xxxx
            bits.set(7); // FIXME: check this
            break;
        }
    }

    const std::unordered_map<std::pair<DriveState, DriveTransition>, DriveState, pair_hash> transitionToState = {
        {{DriveState::SWITCH_ON_DISABLED, DriveTransition::SHUTDOWN}, DriveState::READY_TO_SWITCH_ON}, // 2
        {{DriveState::READY_TO_SWITCH_ON, DriveTransition::SWITCH_ON}, DriveState::SWITCHED_ON}, // 3
        {{DriveState::SWITCHED_ON, DriveTransition::ENABLE_OPERATION}, DriveState::OPERATION_ENABLED}, // 4
        {{DriveState::OPERATION_ENABLED, DriveTransition::DISABLE_OPERATION}, DriveState::SWITCHED_ON}, // 5
        //{{DriveState::OPERATION_ENABLED, DriveTransition::SWITCH_ON}, DriveState::SWITCHED_ON}, // 5 (alias)
        {{DriveState::SWITCHED_ON, DriveTransition::SHUTDOWN}, DriveState::READY_TO_SWITCH_ON}, // 6
        {{DriveState::READY_TO_SWITCH_ON, DriveTransition::DISABLE_VOLTAGE}, DriveState::SWITCH_ON_DISABLED}, // 7
        {{DriveState::OPERATION_ENABLED, DriveTransition::SHUTDOWN}, DriveState::READY_TO_SWITCH_ON}, // 8
        {{DriveState::OPERATION_ENABLED, DriveTransition::DISABLE_VOLTAGE}, DriveState::SWITCH_ON_DISABLED}, // 9
        {{DriveState::SWITCHED_ON, DriveTransition::DISABLE_VOLTAGE}, DriveState::SWITCH_ON_DISABLED}, // 10
        {{DriveState::OPERATION_ENABLED, DriveTransition::QUICK_STOP}, DriveState::QUICK_STOP_ACTIVE}, // 11
        {{DriveState::QUICK_STOP_ACTIVE, DriveTransition::DISABLE_VOLTAGE}, DriveState::SWITCH_ON_DISABLED}, // 12
        {{DriveState::QUICK_STOP_ACTIVE, DriveTransition::ENABLE_OPERATION}, DriveState::OPERATION_ENABLED}, // 16
    };

    const std::unordered_map<std::pair<DriveState, DriveState>, std::vector<DriveTransition>, pair_hash> shortestPaths = {
        {{DriveState::SWITCH_ON_DISABLED, DriveState::READY_TO_SWITCH_ON}, {DriveTransition::SHUTDOWN}},
        {{DriveState::SWITCH_ON_DISABLED, DriveState::SWITCHED_ON}, {DriveTransition::SHUTDOWN, DriveTransition::SWITCH_ON}},
        {{DriveState::SWITCH_ON_DISABLED, DriveState::OPERATION_ENABLED}, {DriveTransition::SHUTDOWN, DriveTransition::SWITCH_ON, DriveTransition::ENABLE_OPERATION}},
        {{DriveState::READY_TO_SWITCH_ON, DriveState::SWITCHED_ON}, {DriveTransition::SWITCH_ON}},
        {{DriveState::READY_TO_SWITCH_ON, DriveState::OPERATION_ENABLED}, {DriveTransition::SWITCH_ON, DriveTransition::ENABLE_OPERATION}},
        {{DriveState::SWITCHED_ON, DriveState::OPERATION_ENABLED}, {DriveTransition::ENABLE_OPERATION}},
        {{DriveState::OPERATION_ENABLED, DriveState::SWITCHED_ON}, {DriveTransition::DISABLE_OPERATION}},
        //{{DriveState::OPERATION_ENABLED, DriveState::SWITCHED_ON}, {DriveTransition::SWITCH_ON}}, // alias
        {{DriveState::OPERATION_ENABLED, DriveState::READY_TO_SWITCH_ON}, {DriveTransition::SHUTDOWN}},
        {{DriveState::OPERATION_ENABLED, DriveState::SWITCH_ON_DISABLED}, {DriveTransition::DISABLE_VOLTAGE}},
        {{DriveState::SWITCHED_ON, DriveState::READY_TO_SWITCH_ON}, {DriveTransition::SHUTDOWN}},
        {{DriveState::SWITCHED_ON, DriveState::SWITCH_ON_DISABLED}, {DriveTransition::DISABLE_VOLTAGE}},
        {{DriveState::READY_TO_SWITCH_ON, DriveState::SWITCH_ON_DISABLED}, {DriveTransition::DISABLE_VOLTAGE}},
        {{DriveState::QUICK_STOP_ACTIVE, DriveState::SWITCH_ON_DISABLED}, {DriveTransition::DISABLE_VOLTAGE}}
    };
}

bool DriveStatusMachine::update(std::uint16_t statusword)
{
    static const word_t mask("0000000001101111"); // state machine-related bits
    word_t _old;
    word_t _new = statusword;

    {
        std::lock_guard<std::mutex> lock(stateMutex);
        _old = _statusword;
        _statusword = _new;
    }

    _old &= mask;
    _new &= mask;

    return _old == _new ? stateObserver.notify() : true;
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

bool DriveStatusMachine::requestState(DriveState goalState)
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
        if (!requestTransition(transition, true))
        {
            return false;
        }
    }

    return true;
}

DriveState DriveStatusMachine::parseStatusword(std::uint16_t statusword)
{
    return parseDriveState(statusword);
}
