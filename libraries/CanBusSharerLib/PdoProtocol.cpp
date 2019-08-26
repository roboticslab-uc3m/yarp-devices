// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "PdoProtocol.hpp"

#include <ColorDebug.h>

using namespace roboticslab;

PdoConfiguration & PdoConfiguration::setValid(bool value)
{
    valid = value;
    return *this;
}

PdoConfiguration & PdoConfiguration::setRtr(bool value)
{
    rtr = value;
    return *this;
}

PdoConfiguration & PdoConfiguration::setTransmissionType(std::uint8_t value)
{
    transmissionType = value;
    return *this;
}

PdoConfiguration & PdoConfiguration::setInhibitTime(std::uint16_t value)
{
    inhibitTime = value;
    return *this;
}

PdoConfiguration & PdoConfiguration::setEventTimer(std::uint16_t value)
{
    eventTimer = value;
    return *this;
}

PdoConfiguration & PdoConfiguration::setSyncStartValue(std::uint8_t value)
{
    syncStartValue = value;
    return *this;
}

bool ConcreteReceivePdo::writeInternal(const std::uint8_t * data, std::size_t size)
{
    return sender->prepareMessage(message_builder(cob + id, size, data));
}

bool InvalidReceivePdo::writeInternal(const std::uint8_t * data, std::size_t size)
{
    CD_ERROR("Invalid RPDO.");
    return false;
}
