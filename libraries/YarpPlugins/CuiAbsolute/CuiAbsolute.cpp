// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CuiAbsolute.hpp"

#include <ColorDebug.h>

#include "CanUtils.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool CuiAbsolute::performRequest(const std::string & name, std::size_t len, const std::uint8_t * data, double * v)
{
    if (!sender->prepareMessage({canId, len, data}))
    {
        CD_ERROR("Unable to send \"%s\" command. %s\n", name.c_str(), CanUtils::msgToStr(canId, 0, len, data).c_str());
    }

    CD_ERROR("Sent \"%s\" command. %s\n", name.c_str(), CanUtils::msgToStr(canId, 0, len, data).c_str());

    bool await = v ? pollStateObserver->await(v) : pushStateObserver->await();

    if (!await)
    {
        CD_ERROR("Command \"%s\" timed out.\n", name.c_str());
        return false;
    }

    CD_SUCCESS("Succesfully processed \"%s\" command.\n", name.c_str());
    return true;
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

bool CuiAbsolute::pollEncoderRead(double * enc)
{
    const std::uint8_t msgData[] = {static_cast<std::uint8_t>(CuiCommand::POLL)};
    return performRequest("poll", 1, msgData, enc);
}

// ------------------------------------------------------------------------------
