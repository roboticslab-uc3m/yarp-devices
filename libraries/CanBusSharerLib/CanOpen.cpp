// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanOpen.hpp"

#include <ColorDebug.h>

using namespace roboticslab;

namespace
{
    // TODO: const?
    InvalidSdoClient INVALID_SDO;
    InvalidReceivePdo INVALID_RPDO;
    InvalidTransmitPdo INVALID_TPDO;
}

CanOpen::CanOpen(int _id)
    : id(_id),
      sender(nullptr),
      hasSender(false)
{
    //createSdo(); // TODO
}

CanOpen::CanOpen(int _id, CanSenderDelegate * _sender)
    : id(_id),
      sender(_sender),
      hasSender(true)
{
    //createSdo(); // TODO
}

CanOpen::~CanOpen()
{
    for (auto it : sdos)
    {
        delete it.second;
    }

    for (auto it : rpdos)
    {
        delete it.second;
    }

    for (auto it : tpdos)
    {
        delete it.second;
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
    if (n < 1 || n > 256)
    {
        CD_ERROR("Illegal RPDO index: %d.\n", n);
        return false;
    }

    const std::uint8_t idx = n - 1;

    if (rpdos.find(idx) == rpdos.end())
    {
        rpdos.insert(std::make_pair(idx, new ConcreteReceivePdo(cob + id)));
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
    if (n < 1 || n > 256)
    {
        CD_ERROR("Illegal TPDO index: %d.\n", n);
        return false;
    }

    const std::uint8_t idx = n - 1;

    if (tpdos.find(idx) == tpdos.end())
    {
        tpdos.insert(std::make_pair(idx, new ConcreteTransmitPdo(cob + id)));
        return true;
    }
    else
    {
        CD_WARNING("TPDO %d already created.\n", n);
        return false;
    }
}
