// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CuiAbsolute.hpp"

#include <yarp/os/Log.h>

#include "CanUtils.hpp"
#include "LogComponent.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool CuiAbsolute::performRequest(const std::string & name, unsigned int len, const std::uint8_t * data, encoder_t * v)
{
    const can_message msg {canId, len, data};
    retry = 0;

    while (++retry <= maxRetries)
    {
        if (sender && !sender->prepareMessage(msg))
        {
            yCError(CUI, "Unable to register \"%s\" command. %s", name.c_str(), CanUtils::msgToStr(msg).c_str());
            return false;
        }

        yCInfo(CUI, "Registered \"%s\" command (%d/%d). %s", name.c_str(), retry, maxRetries, CanUtils::msgToStr(msg).c_str());

        if (v ? pollStateObserver->await(v) : pushStateObserver->await())
        {
            yCInfo(CUI, "Succesfully processed \"%s\" command (%d/%d)", name.c_str(), retry, maxRetries);
            normalize(v);
            return true;
        }

        yCWarning(CUI, "Command \"%s\" timed out (%d/%d)", name.c_str(), retry, maxRetries);
    }

    yCError(CUI, "Max number of retries exceeded (%d)", maxRetries);
    return false;
}

// -----------------------------------------------------------------------------

bool CuiAbsolute::startPushMode()
{
    const std::uint8_t msgData[] = {static_cast<std::uint8_t>(CuiCommand::PUSH_START), pushDelay};
    return performRequest("push start", 2, msgData);
}

// ------------------------------------------------------------------------------

bool CuiAbsolute::stopPushMode()
{
    const std::uint8_t msgData[] = {static_cast<std::uint8_t>(CuiCommand::PUSH_STOP)};
    return performRequest("push stop", 1, msgData);
}

// ------------------------------------------------------------------------------

bool CuiAbsolute::pollEncoderRead(encoder_t * enc)
{
    const std::uint8_t msgData[] = {static_cast<std::uint8_t>(CuiCommand::POLL)};
    return performRequest("poll", 1, msgData, enc);
}

// ------------------------------------------------------------------------------

void CuiAbsolute::normalize(encoder_t * v)
{
    if (reverse)
    {
        *v = -(*v);
    }

    if (*v < -180.0)
    {
        *v += 360.0;
    }
    else if (*v > 180.0)
    {
        *v -= 360.0;
    }
}

// ------------------------------------------------------------------------------
