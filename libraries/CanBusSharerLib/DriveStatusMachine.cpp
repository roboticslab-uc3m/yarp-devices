// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "DriveStatusMachine.hpp"

#include <bitset>

using namespace roboticslab;

namespace
{
    DriveState parseDriveState(const std::bitset<16> & bits)
    {
        if (!bits.test(0) && !bits.test(1) && !bits.test(2) && !bits.test(3) && !bits.test(6))
        {
            return DriveState::NOT_READY_TO_SWITCH_ON; // xxxx xxxx x0xx 0000
        }
        else if (!bits.test(0) && !bits.test(1) && !bits.test(2) && !bits.test(3) && bits.test(6))
        {
            return DriveState::SWITCH_ON_DISABLED; // xxxx xxxx x1xx 0000
        }
        else if (bits.test(0) && !bits.test(1) && !bits.test(2) && !bits.test(3) && bits.test(5) && !bits.test(6))
        {
            return DriveState::READY_TO_SWITCH_ON; // xxxx xxxx x01x 0001
        }
        else if (bits.test(0) && bits.test(1) && !bits.test(2) && !bits.test(3) && bits.test(5) && !bits.test(6))
        {
            return DriveState::SWITCHED_ON; // xxxx xxxx x01x 0011
        }
        else if (bits.test(0) && bits.test(1) && bits.test(2) && !bits.test(3) && bits.test(5) && !bits.test(6))
        {
            return DriveState::OPERATION_ENABLED; // xxxx xxxx x01x 0111
        }
        else if (bits.test(0) && bits.test(1) && bits.test(2) && !bits.test(3) && !bits.test(5) && !bits.test(6))
        {
            return DriveState::QUICK_STOP_ACTIVE; // xxxx xxxx x00x 0111
        }
        else if (bits.test(0) && bits.test(1) && bits.test(2) && bits.test(3) && !bits.test(6))
        {
            return DriveState::FAULT_REACTION_ACTIVE; // xxxx xxxx x0xx 1111
        }
        else if (!bits.test(0) && !bits.test(1) && !bits.test(2) && bits.test(3) && !bits.test(6))
        {
            return DriveState::FAULT; // xxxx xxxx x0xx 1000
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
        case DriveTransition::SHUTDOWN:
            bits.reset(0);
            bits.set(1);
            bits.set(2);
            bits.reset(7);
            break;
        case DriveTransition::SWITCH_ON:
        case DriveTransition::DISABLE_OPERATION:
            bits.set(0);
            bits.set(1);
            bits.set(2);
            bits.reset(3);
            bits.reset(7);
            break;
        case DriveTransition::DISABLE_VOLTAGE:
            bits.reset(1);
            bits.reset(7);
            break;
        case DriveTransition::QUICK_STOP:
            bits.set(1);
            bits.reset(2);
            bits.reset(7);
            break;
        case DriveTransition::ENABLE_OPERATION:
            bits.set(0);
            bits.set(1);
            bits.set(2);
            bits.set(3);
            bits.reset(7);
            break;
        case DriveTransition::FAULT_RESET:
            bits.set(7); // FIXME: check this
            break;
        }
    }
}

void DriveStatusMachine::update(std::uint16_t statusword)
{
    std::lock_guard<std::mutex> lock(stateMutex);
    _statusword = statusword;
}

std::bitset<16> & DriveStatusMachine::controlword()
{
    return _controlword;
}

const std::bitset<16> & DriveStatusMachine::statusword() const
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
    prepareDriveTransition(transition, _controlword);
    std::uint16_t data = _controlword.to_ulong();
    return rpdo->write(data);
}
