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

    using word_t = DriveStatusMachine::word_t;

    word_t stateToControlword(DriveState state)
    {
        switch (state)
        {
        case DriveState::SWITCH_ON_DISABLED:
            return static_cast<std::uint16_t>(DriveTransition::DISABLE_VOLTAGE);
        case DriveState::READY_TO_SWITCH_ON:
            return static_cast<std::uint16_t>(DriveTransition::SHUTDOWN);
        case DriveState::SWITCHED_ON:
            return static_cast<std::uint16_t>(DriveTransition::SWITCH_ON); // same as DISABLE_OPERATION
        case DriveState::OPERATION_ENABLED:
            return static_cast<std::uint16_t>(DriveTransition::ENABLE_OPERATION);
        case DriveState::QUICK_STOP_ACTIVE:
            return static_cast<std::uint16_t>(DriveTransition::QUICK_STOP);
        default: // NOT_READY_TO_SWITCH_ON and fault states
            return 0x0000;
        }
    }

    inline word_t updateStateBits(const word_t & stored, const word_t & requested)
    {
        static const word_t controlwordMaskNot = ~word_t("0000000010001111"); // state machine-related bits
        return (stored & controlwordMaskNot) | requested;
    }

    using ds = DriveState;
    using dt = DriveTransition;

    const std::unordered_map<std::pair<DriveState, DriveTransition>, DriveState, pair_hash> nextStateOnTransition = {
        {{ds::SWITCH_ON_DISABLED, dt::SHUTDOWN}, ds::READY_TO_SWITCH_ON}, // 2
        {{ds::READY_TO_SWITCH_ON, dt::SWITCH_ON}, ds::SWITCHED_ON}, // 3
        {{ds::SWITCHED_ON, dt::ENABLE_OPERATION}, ds::OPERATION_ENABLED}, // 4
        {{ds::OPERATION_ENABLED, dt::DISABLE_OPERATION}, ds::SWITCHED_ON}, // 5
        //{{ds::OPERATION_ENABLED, dt::SWITCH_ON}, ds::SWITCHED_ON}, // 5 (alias)
        {{ds::SWITCHED_ON, dt::SHUTDOWN}, ds::READY_TO_SWITCH_ON}, // 6
        {{ds::READY_TO_SWITCH_ON, dt::DISABLE_VOLTAGE}, ds::SWITCH_ON_DISABLED}, // 7
        {{ds::OPERATION_ENABLED, dt::SHUTDOWN}, ds::READY_TO_SWITCH_ON}, // 8
        {{ds::OPERATION_ENABLED, dt::DISABLE_VOLTAGE}, ds::SWITCH_ON_DISABLED}, // 9
        {{ds::SWITCHED_ON, dt::DISABLE_VOLTAGE}, ds::SWITCH_ON_DISABLED}, // 10
        {{ds::OPERATION_ENABLED, dt::QUICK_STOP}, ds::QUICK_STOP_ACTIVE}, // 11
        {{ds::QUICK_STOP_ACTIVE, dt::DISABLE_VOLTAGE}, ds::SWITCH_ON_DISABLED}, // 12
        {{ds::FAULT, dt::FAULT_RESET}, ds::SWITCH_ON_DISABLED}, // 15
        {{ds::QUICK_STOP_ACTIVE, dt::ENABLE_OPERATION}, ds::OPERATION_ENABLED}, // 16
    };

    const std::unordered_map<std::pair<DriveState, DriveState>, std::vector<DriveTransition>, pair_hash> shortestPaths = {
        {{ds::SWITCH_ON_DISABLED, ds::READY_TO_SWITCH_ON}, {dt::SHUTDOWN}},
        {{ds::SWITCH_ON_DISABLED, ds::SWITCHED_ON}, {dt::SHUTDOWN, dt::SWITCH_ON}},
        {{ds::SWITCH_ON_DISABLED, ds::OPERATION_ENABLED}, {dt::SHUTDOWN, dt::SWITCH_ON, dt::ENABLE_OPERATION}},
        {{ds::READY_TO_SWITCH_ON, ds::SWITCHED_ON}, {dt::SWITCH_ON}},
        {{ds::READY_TO_SWITCH_ON, ds::OPERATION_ENABLED}, {dt::SWITCH_ON, dt::ENABLE_OPERATION}},
        {{ds::SWITCHED_ON, ds::OPERATION_ENABLED}, {dt::ENABLE_OPERATION}},
        {{ds::OPERATION_ENABLED, ds::SWITCHED_ON}, {dt::DISABLE_OPERATION}},
        //{{ds::OPERATION_ENABLED, ds::SWITCHED_ON}, {dt::SWITCH_ON}}, // alias
        {{ds::OPERATION_ENABLED, ds::READY_TO_SWITCH_ON}, {dt::SHUTDOWN}},
        {{ds::OPERATION_ENABLED, ds::SWITCH_ON_DISABLED}, {dt::DISABLE_VOLTAGE}},
        {{ds::SWITCHED_ON, ds::READY_TO_SWITCH_ON}, {dt::SHUTDOWN}},
        {{ds::SWITCHED_ON, ds::SWITCH_ON_DISABLED}, {dt::DISABLE_VOLTAGE}},
        {{ds::READY_TO_SWITCH_ON, ds::SWITCH_ON_DISABLED}, {dt::DISABLE_VOLTAGE}},
        {{ds::QUICK_STOP_ACTIVE, ds::SWITCH_ON_DISABLED}, {dt::DISABLE_VOLTAGE}}
    };
}

bool DriveStatusMachine::update(std::uint16_t statusword)
{
    // state machine-related bits (at least those we can transition to)
    static const word_t statuswordMask("0000000001100111");

    std::lock_guard<std::mutex> lock(stateMutex);
    const word_t old = _statusword;
    _statusword = statusword;

    if ((old & statuswordMask) != (_statusword & statuswordMask))
    {
        word_t requested = stateToControlword(parseDriveState(_statusword));
        _controlword = updateStateBits(_controlword, requested);
        return stateObserver.notify();
    }

    return true;
}

void DriveStatusMachine::reset()
{
    std::lock_guard<std::mutex> lock(stateMutex);
    _statusword = 0;
    _controlword = 0;
}

DriveStatusMachine::word_t DriveStatusMachine::controlword() const
{
    std::lock_guard<std::mutex> lock(stateMutex);
    return _controlword;
}

bool DriveStatusMachine::controlword(const word_t & controlbits)
{
    if (!rpdo->write<std::uint16_t>(controlbits.to_ulong()))
    {
        return false;
    }

    std::lock_guard<std::mutex> lock(stateMutex);
    _controlword = controlbits;
    return true;
}

DriveStatusMachine::word_t DriveStatusMachine::statusword() const
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
    auto it = nextStateOnTransition.find({initialState, transition});
    word_t requested = updateStateBits(controlword(), static_cast<std::uint16_t>(transition));

    return it != nextStateOnTransition.cend()
            && controlword(requested)
            && (wait ? awaitState(it->second) : true);
}

bool DriveStatusMachine::requestState(DriveState goalState)
{
    DriveState initialState = getCurrentState();

    if (goalState == initialState)
    {
        return true;
    }

    auto it = shortestPaths.find({initialState, goalState});

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

bool DriveStatusMachine::awaitState(DriveState goalState)
{
    if (goalState == getCurrentState())
    {
        return true;
    }

    return stateObserver.await() && goalState == getCurrentState();
}

DriveState DriveStatusMachine::parseStatusword(std::uint16_t statusword)
{
    return parseDriveState(statusword);
}
