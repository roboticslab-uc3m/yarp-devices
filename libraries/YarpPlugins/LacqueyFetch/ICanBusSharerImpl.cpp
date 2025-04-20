// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "LacqueyFetch.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

unsigned int LacqueyFetch::getId()
{
    return m_canId;
}

// -----------------------------------------------------------------------------

bool LacqueyFetch::initialize()
{
    return true;
}

// -----------------------------------------------------------------------------

bool LacqueyFetch::finalize()
{
    return true;
}

// -----------------------------------------------------------------------------

bool LacqueyFetch::notifyMessage(const can_message & message)
{
    return true;
}

// -----------------------------------------------------------------------------

bool LacqueyFetch::registerSender(ICanSenderDelegate * sender)
{
    this->sender = sender;
    return true;
}

// -----------------------------------------------------------------------------

bool LacqueyFetch::synchronize(double timestamp)
{
    return true;
}

// -----------------------------------------------------------------------------
