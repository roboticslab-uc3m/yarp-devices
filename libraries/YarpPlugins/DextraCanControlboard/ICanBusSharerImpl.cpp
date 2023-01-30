// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "DextraCanControlboard.hpp"

using namespace roboticslab;

unsigned int DextraCanControlboard::getId()
{
    return canId;
}

bool DextraCanControlboard::initialize()
{
    return true;
}

bool DextraCanControlboard::finalize()
{
    return true;
}

bool DextraCanControlboard::registerSender(ICanSenderDelegate * sender)
{
    synapse->configure(sender);
    return true;
}

bool DextraCanControlboard::notifyMessage(const can_message & message)
{
    return true;
}

bool DextraCanControlboard::synchronize(double timestamp)
{
    return true;
}
