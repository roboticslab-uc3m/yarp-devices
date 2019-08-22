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
