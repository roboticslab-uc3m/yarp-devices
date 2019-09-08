// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "PdoProtocol.hpp"

#include <bitset>

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

bool ReceivePdo::writeInternal(const std::uint8_t * data, std::size_t size)
{
    return sender->prepareMessage(message_builder(cob + id, size, data));
}

bool PdoProtocol::configure(const PdoConfiguration & conf)
{
    std::string pdoType;
    std::uint16_t commIdx;
    std::uint16_t mappingIdx;

    switch (getType())
    {
    case PdoType::RPDO:
        pdoType = "RPDO" + std::to_string(n);
        commIdx = 0x1400 + n - 1;
        mappingIdx = 0x1600 + n - 1;
        break;
    case PdoType::TPDO:
        pdoType = "TPDO" + std::to_string(n);
        commIdx = 0x1800 + n - 1;
        mappingIdx = 0x1A00 + n - 1;
        break;
    default:
        CD_ERROR("Unknown PDO type.\n");
        return false;
    }

    std::uint32_t cobId;

    if (!sdo->upload(std::string("COB-ID ") + pdoType, &cobId, commIdx, 0x01))
    {
        return false;
    }

    std::bitset<32>bits(cobId);
    bits.set(31);

    if (conf.rtr)
    {
        if (getType() != PdoType::TPDO)
        {
            CD_ERROR("Illegal RTR usage on non-TPDO node.\n");
            return false;
        }

        bits.set(30, !*conf.rtr);
    }

    if (!sdo->download(std::string("COB-ID ") + pdoType, bits.to_ulong(), commIdx, 0x01))
    {
        return false;
    }

    if (conf.transmissionType && !sdo->download("Transmission type", *conf.transmissionType, commIdx, 0x02))
    {
        return false;
    }

    if (conf.inhibitTime && !sdo->download("Inhibit time", *conf.inhibitTime, commIdx, 0x03))
    {
        return false;
    }

    if (conf.eventTimer && !sdo->download("Event timer", *conf.eventTimer, commIdx, 0x05))
    {
        return false;
    }

    if (conf.syncStartValue)
    {
        if (getType() != PdoType::TPDO)
        {
            CD_ERROR("Illegal SYNC start value usage on non-TPDO node.\n");
            return false;
        }

        if (!sdo->download("SYNC start value", *conf.syncStartValue, commIdx, 0x06))
        {
            return false;
        }
    }

    if (!conf.mappings.empty())
    {
        if (!sdo->download<std::uint8_t>(pdoType + " mapping parameters", 0, mappingIdx))
        {
            return false;
        }

        unsigned int i = 0;

        for (auto mapping : conf.mappings)
        {
            i++;
            std::string name = pdoType + ": mapped object " + std::to_string(i);

            if (!sdo->download(name, mapping, mappingIdx, i))
            {
                return false;
            }
        }

        if (!sdo->download<std::uint8_t>(pdoType + " mapping parameters", i, mappingIdx))
        {
            return false;
        }
    }

    if (!conf.valid || *conf.valid)
    {
        bits.reset(31);

        if (!sdo->download(std::string("COB-ID ") + pdoType, bits.to_ulong(), commIdx, 0x01))
        {
            return false;
        }
    }

    return true;
}
