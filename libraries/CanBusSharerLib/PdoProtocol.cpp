// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "PdoProtocol.hpp"

#include <ColorDebug.h>

using namespace roboticslab;

bool ConcreteReceivePdo::writeInternal(const std::uint8_t * data, std::size_t size)
{
    return true;
}

bool InvalidReceivePdo::writeInternal(const std::uint8_t * data, std::size_t size)
{
    CD_ERROR("Invalid RPDO.");
    return false;
}
