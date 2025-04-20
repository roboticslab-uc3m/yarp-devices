// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CuiAbsolute.hpp"

#include <yarp/os/Log.h>

#include "CanUtils.hpp"
#include "LogComponent.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool CuiAbsolute::performRequest(const std::string & name, unsigned int len, const std::uint8_t * data, encoder_t * v)
{
    const can_message msg {static_cast<std::uint8_t>(m_canId), len, data};
    int retry = 0;

    while (++retry <= m_maxRetries)
    {
        if (sender && !sender->prepareMessage(msg))
        {
            yCIError(CUI, id(), "Unable to register \"%s\" command: %s", name.c_str(), CanUtils::msgToStr(len, data).c_str());
            return false;
        }

        yCIInfo(CUI, id(), "Registered \"%s\" command (%d/%d): %s", name.c_str(), retry, m_maxRetries, CanUtils::msgToStr(len, data).c_str());

        if (v ? pollStateObserver->await(v) : pushStateObserver->await())
        {
            yCIInfo(CUI, id(), "Successfully processed \"%s\" command (%d/%d)", name.c_str(), retry, m_maxRetries);
            normalize(v);
            return true;
        }

        yCIWarning(CUI, id(), "Command \"%s\" timed out (%d/%d)", name.c_str(), retry, m_maxRetries);
    }

    yCIError(CUI, id(), "Max number of retries exceeded (%d)", m_maxRetries);
    return false;
}

// -----------------------------------------------------------------------------

bool CuiAbsolute::startPushMode()
{
    const std::uint8_t msgData[] = {static_cast<std::uint8_t>(CuiCommand::PUSH_START), static_cast<std::uint8_t>(m_pushDelay)};
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
    if (m_reverse)
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
