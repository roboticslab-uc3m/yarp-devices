// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CuiAbsolute.hpp"

#include <cstring>

#include <yarp/os/Time.h>

#include <ColorDebug.h>

#include "CanUtils.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

unsigned int CuiAbsolute::getId()
{
    return canId;
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

bool CuiAbsolute::start()
{
    return true;
}

// -----------------------------------------------------------------------------

bool CuiAbsolute::readyToSwitchOn()
{
    return true;
}

// -----------------------------------------------------------------------------

bool CuiAbsolute::switchOn()
{
    return true;
}

// -----------------------------------------------------------------------------

bool CuiAbsolute::enable()
{
    return true;
}

// -----------------------------------------------------------------------------

bool CuiAbsolute::recoverFromError()
{
    return true;
}

// ------------------------------------------------------------------------------

bool CuiAbsolute::interpretMessage(const yarp::dev::CanMessage & message)
{
    if (message.getData()[3] == 0xc4)
    {
        CD_ERROR_NO_HEADER("Known PIC error. %s\n", CanUtils::msgToStr(message).c_str());
        return false;
    }

    std::uint16_t op = message.getId() - canId;

    switch (op)
    {
    case 0x80: // push mode streaming data
        {
            std::lock_guard<std::mutex> lock(mutex);
            std::memcpy(&encoder, message.getData(), message.getLen()); // getLen() = 4 bytes

            if (reverse)
            {
                encoder = -encoder;
            }

            if (encoder < -180.0)
            {
                encoder += 360.0;
            }
            else if (encoder > 180.0)
            {
                encoder -= 360.0;
            }

            encoderTimestamp = yarp::os::Time::now();
        }
        break;
    case 0x100: // start/stop push mode
        pushStateObserver->notify();
        break;
    case 0x180: // polling
        pollStateObserver->notify(message.getData(), message.getLen());
        break;
    default:
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------------

bool CuiAbsolute::registerSender(CanSenderDelegate * sender)
{
    this->sender = sender;
    return true;
}

// -----------------------------------------------------------------------------
