// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanOpen.hpp"

using namespace roboticslab;

#include <ColorDebug.h>

CanOpen::CanOpen(int _id)
    : id(_id),
      _sdo(nullptr)
{}

CanOpen::~CanOpen()
{
    delete _sdo;

    for (auto it : rpdos)
    {
        delete it.second;
    }
}

SdoClient * CanOpen::sdo()
{
    if (_sdo != nullptr)
    {
        return _sdo;
    }
    else
    {
        return new InvalidSdoClient;
    }
}

bool CanOpen::configureSdo(double timeout)
{
    if (_sdo == nullptr)
    {
        _sdo = new ConcreteSdoClient(id, timeout);
        return true;
    }
    else
    {
        CD_WARNING("SDO client already configured.\n");
        return false;
    }
}

ReceivePdo * CanOpen::rpdo(unsigned int n)
{
    auto it = rpdos.find(n);

    if (rpdos.find(n) != rpdos.end())
    {
        return it->second;
    }
    else
    {
        return new InvalidReceivePdo;
    }
}

bool CanOpen::configureRpdo(unsigned int n)
{
    if (rpdos.find(n) == rpdos.end())
    {
        rpdos.insert(std::make_pair(n, new ConcreteReceivePdo(id)));
        return true;
    }
    else
    {
        CD_WARNING("RPDO %d already configured.\n", n);
        return false;
    }
}
