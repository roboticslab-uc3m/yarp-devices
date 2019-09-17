// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CuiAbsolute.hpp"

#include <ColorDebug.h>

#include "CanUtils.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool CuiAbsolute::send(std::uint16_t len, std::uint8_t * msgData)
{
    return sender->prepareMessage(message_builder(canId, len, msgData));
}

// -----------------------------------------------------------------------------

bool CuiAbsolute::startContinuousPublishing(std::uint8_t delay)
{
    std::uint8_t msgData[] = {0x01, 0x01, delay, 0x00, 0x00, 0x00, 0x00, 0x00};

    if (!send(8, msgData))
    {
        CD_ERROR("Could not send \"startContinuousPublishing\" to Cui Absolute Encoder. %s\n", CanUtils::msgToStr(canId, 0, 8, msgData).c_str());
        return false;
    }

    CD_SUCCESS("Sent \"startContinuousPublishing\" to Cui Absolute Encoder. %s\n", CanUtils::msgToStr(canId, 0, 8, msgData).c_str());
    return true;
}

// ------------------------------------------------------------------------------

bool CuiAbsolute::startPullPublishing()
{
    std::uint8_t msgData[] = {0x01, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

    if (!send(8, msgData))
    {
        CD_ERROR("Could not send \"startPullPublishing\" to Cui Absolute Encoder. %s\n", CanUtils::msgToStr(canId, 0, 8, msgData).c_str());
        return false;
    }

    CD_SUCCESS("Sent \"startPullPublishing\" to Cui Absolute Encoder. %s\n", CanUtils::msgToStr(canId, 0, 8, msgData).c_str());
    return true;
}

// ------------------------------------------------------------------------------

bool CuiAbsolute::stopPublishingMessages()
{
    std::uint8_t msgData[] = {0x02, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

    if (!send(8, msgData))
    {
        CD_ERROR("Could not send \"stopPublishingMessages\" to Cui Absolute Encoder. %s\n", CanUtils::msgToStr(canId, 0, 8, msgData).c_str());
        return false;
    }

    CD_SUCCESS("Sent \"stopPublishingMessages\" to Cui Absolute Encoder. %s\n", CanUtils::msgToStr(canId, 0, 8, msgData).c_str());
    return true;
}

// ------------------------------------------------------------------------------
