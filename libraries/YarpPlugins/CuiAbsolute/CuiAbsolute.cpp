// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CuiAbsolute.hpp"

#include <ColorDebug.h>

#include "CanUtils.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool CuiAbsolute::performRequest(const std::string & name, unsigned int len, const std::uint8_t * data, encoder_t * v)
{
    retry = 0;

    while (++retry <= maxRetries)
    {
        if (!sender->prepareMessage({canId, len, data}))
        {
            CD_ERROR("Unable to register \"%s\" command. %s\n", name.c_str(), CanUtils::msgToStr(canId, 0, len, data).c_str());
            return false;
        }

        CD_INFO("Registered \"%s\" command (%d/%d). %s\n", name.c_str(), retry, maxRetries, CanUtils::msgToStr(canId, 0, len, data).c_str());

        if (v ? pollStateObserver->await(v) : pushStateObserver->await())
        {
            CD_SUCCESS("Succesfully processed \"%s\" command (%d/%d).\n", name.c_str(), retry, maxRetries);
            return true;
        }

        CD_WARNING("Command \"%s\" timed out (%d/%d).\n", name.c_str(), retry, maxRetries);
    }

    CD_ERROR("Max number of retries exceeded (%d).\n", maxRetries);
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
