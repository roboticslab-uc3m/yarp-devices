// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "PdoProtocol.hpp"

#include <cstring>

#include <bitset>
#include <optional>
#include <vector>

#include <yarp/os/LogComponent.h>
#include <yarp/os/LogStream.h>

using namespace roboticslab;

namespace
{
    YARP_LOG_COMPONENT(PDO, "rl.PdoProtocol")
}

struct PdoConfiguration::Private
{
    std::optional<bool> valid;
    std::optional<bool> rtr;
    std::optional<PdoTransmissionType> transmissionType;
    std::optional<std::uint16_t> inhibitTime;
    std::optional<std::uint16_t> eventTimer;
    std::optional<std::uint8_t> syncStartValue;
    std::vector<std::uint32_t> mappings;
};

PdoConfiguration::PdoConfiguration()
    : priv(new Private)
{ }

PdoConfiguration::~PdoConfiguration()
{
    delete priv;
}

PdoConfiguration::PdoConfiguration(const PdoConfiguration & other)
    : priv(new Private(*other.priv))
{ }

PdoConfiguration & PdoConfiguration::operator=(const PdoConfiguration & other)
{
    if (this != &other)
    {
        delete priv;
        priv = new Private(*other.priv);
    }

    return *this;
}

PdoConfiguration & PdoConfiguration::setValid(bool value)
{
    priv->valid = value;
    return *this;
}

PdoConfiguration & PdoConfiguration::setRtr(bool value)
{
    priv->rtr = value;
    return *this;
}

PdoConfiguration & PdoConfiguration::setTransmissionType(PdoTransmissionType value)
{
    priv->transmissionType = value;
    return *this;
}

PdoConfiguration & PdoConfiguration::setInhibitTime(std::uint16_t value)
{
    priv->inhibitTime = value;
    return *this;
}

PdoConfiguration & PdoConfiguration::setEventTimer(std::uint16_t value)
{
    priv->eventTimer = value;
    return *this;
}

PdoConfiguration & PdoConfiguration::setSyncStartValue(std::uint8_t value)
{
    priv->syncStartValue = value;
    return *this;
}

void PdoConfiguration::addMappingInternal(std::uint32_t value)
{
    priv->mappings.push_back(value);
}

bool PdoProtocol::configure(const PdoConfiguration & conf)
{
    std::string pdoType;
    std::uint16_t commIdx;
    std::uint16_t mappingIdx;

    std::string logId = "ID" + std::to_string(id);

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
        yCIError(PDO, logId) << "Unknown PDO type";
        return false;
    }

    logId += "/" + pdoType;

    std::uint32_t cobId;

    if (!sdo->upload(std::string("COB-ID ") + pdoType, &cobId, commIdx, 0x01))
    {
        return false;
    }

    std::bitset<32>bits(cobId);
    bits.set(31);

    if (conf.priv->rtr)
    {
        if (getType() != PdoType::TPDO)
        {
            yCIError(PDO, logId) << "Illegal RTR usage on non-TPDO node";
            return false;
        }

        bits.set(30, !*conf.priv->rtr);
    }

    if (!sdo->download<std::uint32_t>(std::string("COB-ID ") + pdoType, bits.to_ulong(), commIdx, 0x01))
    {
        return false;
    }

    if (conf.priv->transmissionType && !sdo->download("Transmission type", static_cast<std::uint8_t>(*conf.priv->transmissionType), commIdx, 0x02))
    {
        return false;
    }

    if (conf.priv->inhibitTime && !sdo->download("Inhibit time", *conf.priv->inhibitTime, commIdx, 0x03))
    {
        return false;
    }

    if (conf.priv->eventTimer && !sdo->download("Event timer", *conf.priv->eventTimer, commIdx, 0x05))
    {
        return false;
    }

    if (conf.priv->syncStartValue)
    {
        if (getType() != PdoType::TPDO)
        {
            yCIError(PDO, logId) << "Illegal SYNC start value usage on non-TPDO node";
            return false;
        }

        if (!sdo->download("SYNC start value", *conf.priv->syncStartValue, commIdx, 0x06))
        {
            return false;
        }
    }

    if (!conf.priv->mappings.empty())
    {
        if (!sdo->download<std::uint8_t>(pdoType + " mapping parameters", 0, mappingIdx))
        {
            return false;
        }

        unsigned int i = 0;

        for (auto mapping : conf.priv->mappings)
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

    if (!conf.priv->valid || *conf.priv->valid)
    {
        bits.reset(31);

        if (!sdo->download<std::uint32_t>(std::string("COB-ID ") + pdoType, bits.to_ulong(), commIdx, 0x01))
        {
            return false;
        }
    }

    return true;
}

void ReceivePdo::packInternal(std::uint8_t * buff, const void * data, unsigned int size)
{
    std::memcpy(buff, data, size);
}

bool ReceivePdo::writeInternal(const std::uint8_t * data, unsigned int size)
{
    return sender && sender->prepareMessage({getCobId(), size, data});
}

void TransmitPdo::unpackInternal(void * data, const std::uint8_t * buff, unsigned int size)
{
    std::memcpy(data, buff, size);
}
