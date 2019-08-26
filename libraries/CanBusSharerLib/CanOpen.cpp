// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanOpen.hpp"

#include <bitset>

#include <ColorDebug.h>

using namespace roboticslab;

namespace
{
    // TODO: const?
    InvalidSdoClient INVALID_SDO;
    InvalidReceivePdo INVALID_RPDO;
    InvalidTransmitPdo INVALID_TPDO;
    InvalidEmcyConsumer INVALID_EMCY;
}

CanOpen::CanOpen(int _id)
    : id(_id),
      sender(nullptr),
      hasSender(false),
      _emcy(nullptr)
{
    //createSdo(); // TODO
}

CanOpen::CanOpen(int _id, CanSenderDelegate * _sender)
    : id(_id),
      sender(_sender),
      hasSender(true),
      _emcy(nullptr)
{
    //createSdo(); // TODO
}

CanOpen::~CanOpen()
{
    for (auto entry : sdos)
    {
        delete entry.second;
    }

    for (auto entry : rpdos)
    {
        delete entry.second;
    }

    for (auto entry : tpdos)
    {
        delete entry.second;
    }
}

void CanOpen::configureSender(CanSenderDelegate * _sender) // TODO
{
    sender = _sender;
    hasSender = true;
}

SdoClient * CanOpen::sdo() const
{
    return sdo(1);
}

SdoClient * CanOpen::sdo(unsigned int n) const
{
    const std::uint8_t idx = n - 1;
    auto it = sdos.find(idx);
    return it != sdos.end() ? it->second : &INVALID_SDO;
}

bool CanOpen::createSdo(double timeout)
{
    return createSdo(1, 0x600, 0x580, timeout);
}

bool CanOpen::createSdo(unsigned int n, std::uint16_t cobRx, std::uint16_t cobTx, double timeout)
{
    if (n < 1 || n > 128)
    {
        CD_ERROR("Illegal SDO index: %d.\n", n);
        return false;
    }

    const std::uint8_t idx = n - 1;

    if (sdos.find(idx) == sdos.end())
    {
        sdos.insert(std::make_pair(idx, new ConcreteSdoClient(id, cobRx, cobTx, timeout)));
        return true;
    }
    else
    {
        CD_WARNING("SDO client %d already created.\n", n);
        return false;
    }
}

ReceivePdo * CanOpen::rpdo1() const
{
    return rpdo(1);
}

ReceivePdo * CanOpen::rpdo2() const
{
    return rpdo(2);
}

ReceivePdo * CanOpen::rpdo3() const
{
    return rpdo(3);
}

ReceivePdo * CanOpen::rpdo4() const
{
    return rpdo(4);
}

ReceivePdo * CanOpen::rpdo(unsigned int n) const
{
    const std::uint8_t idx = n - 1;
    auto it = rpdos.find(idx);
    return it != rpdos.end() ? it->second : &INVALID_RPDO;
}

bool CanOpen::createRpdo1()
{
    return createRpdo(1, 0x200);
}

bool CanOpen::createRpdo2()
{
    return createRpdo(2, 0x300);
}

bool CanOpen::createRpdo3()
{
    return createRpdo(3, 0x400);
}

bool CanOpen::createRpdo4()
{
    return createRpdo(4, 0x500);
}

bool CanOpen::createRpdo(unsigned int n, std::uint16_t cob)
{
    if (n < 1 || n > 512)
    {
        CD_ERROR("Illegal RPDO index: %d.\n", n);
        return false;
    }

    const std::uint8_t idx = n - 1;

    if (rpdos.find(idx) == rpdos.end())
    {
        rpdos.insert(std::make_pair(idx, new ConcreteReceivePdo(id, cob)));
        return true;
    }
    else
    {
        CD_WARNING("RPDO %d already created.\n", n);
        return false;
    }
}

TransmitPdo * CanOpen::tpdo1() const
{
    return tpdo(1);
}

TransmitPdo * CanOpen::tpdo2() const
{
    return tpdo(2);
}

TransmitPdo * CanOpen::tpdo3() const
{
    return tpdo(3);
}

TransmitPdo * CanOpen::tpdo4() const
{
    return tpdo(4);
}

TransmitPdo * CanOpen::tpdo(unsigned int n) const
{
    const std::uint8_t idx = n - 1;
    auto it = tpdos.find(idx);
    return it != tpdos.end() ? it->second : &INVALID_TPDO;
}

bool CanOpen::createTpdo1()
{
    return createTpdo(1, 0x180);
}

bool CanOpen::createTpdo2()
{
    return createTpdo(2, 0x280);
}

bool CanOpen::createTpdo3()
{
    return createTpdo(3, 0x380);
}

bool CanOpen::createTpdo4()
{
    return createTpdo(4, 0x480);
}

bool CanOpen::createTpdo(unsigned int n, std::uint16_t cob)
{
    if (n < 1 || n > 512)
    {
        CD_ERROR("Illegal TPDO index: %d.\n", n);
        return false;
    }

    const std::uint8_t idx = n - 1;

    if (tpdos.find(idx) == tpdos.end())
    {
        tpdos.insert(std::make_pair(idx, new ConcreteTransmitPdo(id, cob)));
        return true;
    }
    else
    {
        CD_WARNING("TPDO %d already created.\n", n);
        return false;
    }
}

bool CanOpen::configureRpdo(unsigned int n, const PdoConfiguration & conf)
{
    return configurePdo(n, conf, false);
}

bool CanOpen::configureTpdo(unsigned int n, const PdoConfiguration & conf)
{
    return configurePdo(n, conf, true);
}

bool CanOpen::configurePdo(unsigned int n, const PdoConfiguration & conf, bool isTpdo)
{
    std::string pdoType = isTpdo ? "TPDO" : "RPDO";
    std::string pdoName = pdoType + std::to_string(n);

    if (n < 1 || n > 512)
    {
        CD_ERROR("Illegal %s index: %d.\n", pdoType.c_str(), n);
        return false;
    }

    const std::uint8_t idx = n - 1;
    const std::uint16_t commIdx = (isTpdo  ? 0x1800 : 0x1400) + idx;
    const std::uint16_t mappingIdx = (isTpdo ? 0x1A00 : 0x1600) + idx;

    std::uint32_t cobId;

    if (!sdo()->upload(std::string("COB-ID ") + pdoName, &cobId, commIdx, 0x01))
    {
        return false;
    }

    std::bitset<32>bits(cobId);
    bits.set(31);

    if (conf.rtr)
    {
        if (!isTpdo)
        {
            CD_ERROR("Illegal RTR usage on non-TPDO node.\n");
            return false;
        }

        bits.set(30, !*conf.rtr);
    }

    if (!sdo()->download(std::string("COB-ID ") + pdoName, bits.to_ulong(), commIdx, 0x01))
    {
        return false;
    }

    if (conf.transmissionType && !sdo()->download("Transmission type", *conf.transmissionType, commIdx, 0x02))
    {
        return false;
    }

    if (conf.inhibitTime && !sdo()->download("Inhibit time", *conf.inhibitTime, commIdx, 0x03))
    {
        return false;
    }

    if (conf.eventTimer && !sdo()->download("Event timer", *conf.eventTimer, commIdx, 0x05))
    {
        return false;
    }

    if (conf.syncStartValue)
    {
        if (!isTpdo)
        {
            CD_ERROR("Illegal SYNC start value usage on non-TPDO node.\n");
            return false;
        }

        if (!sdo()->download("SYNC start value", *conf.syncStartValue, commIdx, 0x06))
        {
            return false;
        }
    }

    if (!conf.mappings.empty())
    {
        if (!sdo()->download<std::uint8_t>(pdoName + " mapping parameters", 0, mappingIdx))
        {
            return false;
        }

        unsigned int i = 0;

        for (auto mapping : conf.mappings)
        {
            i++;
            std::string name = pdoName + ": mapped object " + std::to_string(i);

            if (!sdo()->download(name, mapping, mappingIdx, i))
            {
                return false;
            }
        }

        if (!sdo()->download<std::uint8_t>(pdoName + " mapping parameters", i, mappingIdx))
        {
            return false;
        }
    }

    if (!conf.valid || *conf.valid)
    {
        bits.reset(31);

        if (!sdo()->download(std::string("COB-ID ") + pdoName, bits.to_ulong(), commIdx, 0x01))
        {
            return false;
        }
    }

    return true;
}

EmcyConsumer * CanOpen::emcy() const
{
    return _emcy != nullptr ? _emcy : &INVALID_EMCY;
}

bool CanOpen::createEmcy()
{
    if (_emcy == nullptr)
    {
        _emcy = new EmcyConsumer;
        return true;
    }
    else
    {
        CD_WARNING("EMCY already created.\n");
        return false;
    }
}
