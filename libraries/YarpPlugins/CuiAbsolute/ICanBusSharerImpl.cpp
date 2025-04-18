// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CuiAbsolute.hpp"

#include <cstring>

#include <yarp/os/LogStream.h>
#include <yarp/os/Time.h>

#include "CanUtils.hpp"
#include "LogComponent.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

unsigned int CuiAbsolute::getId()
{
    return m_canId;
}

// -----------------------------------------------------------------------------

bool CuiAbsolute::initialize()
{
    if (cuiMode == CuiMode::PUSH)
    {
        return startPushMode();
    }

    return true;
}

// -----------------------------------------------------------------------------

bool CuiAbsolute::finalize()
{
    if (cuiMode == CuiMode::PUSH && !stopPushMode())
    {
        yCIError(CUI, id()) << "Unable to stop";
        return false;
    }

    return true;
}

// ------------------------------------------------------------------------------

bool CuiAbsolute::notifyMessage(const can_message & message)
{
    if (message.data[3] == 0xc4)
    {
        yCIError(CUI, id(), "Known PIC error. %s", CanUtils::msgToStr(message).c_str());
        return false;
    }

    std::uint16_t op = message.id - m_canId;

    switch (op)
    {
    case 0x80: // push mode streaming data
    {
        encoder_t v;
        std::memcpy(&v, message.data, message.len); // len = 4 bytes
        normalize(&v);

        std::lock_guard lock(mutex);
        encoder = v;
        encoderTimestamp = yarp::os::Time::now();

        return true;
    }
    case 0x100: // start/stop push mode
        return pushStateObserver->notify();
    case 0x180: // polling
        return pollStateObserver->notify(message.data, message.len);
    default:
        return false;
    }
}

// -----------------------------------------------------------------------------

bool CuiAbsolute::registerSender(ICanSenderDelegate * sender)
{
    this->sender = sender;
    return true;
}

// -----------------------------------------------------------------------------

bool CuiAbsolute:: synchronize(double timestamp)
{
    return true;
}

// -----------------------------------------------------------------------------
