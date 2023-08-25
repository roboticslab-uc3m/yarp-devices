// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "DextraCanControlBoard.hpp"

using namespace roboticslab;

unsigned int DextraCanControlBoard::getId()
{
    return canId;
}

bool DextraCanControlBoard::initialize()
{
    return true;
}

bool DextraCanControlBoard::finalize()
{
    return true;
}

bool DextraCanControlBoard::registerSender(ICanSenderDelegate * sender)
{
    synapse->configure(sender);
    return true;
}

bool DextraCanControlBoard::notifyMessage(const can_message & message)
{
    return true;
}

bool DextraCanControlBoard::synchronize(double timestamp)
{
    return true;
}
