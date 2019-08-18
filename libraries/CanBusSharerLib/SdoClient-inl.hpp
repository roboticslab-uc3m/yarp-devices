// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __SDO_CLIENT_INL_HPP__
#define __SDO_CLIENT_INL_HPP__

#include "SdoClient.hpp"

#include <string>

#include <ColorDebug.h>

#include "CanUtils.hpp"

template<typename T>
bool roboticslab::SdoClient::upload(const std::string & name, T * data, uint16_t index, uint8_t subindex)
{
    uint8_t uploadMsg[4];

    uploadMsg[0] = 0x40;
    std::memcpy(uploadMsg + 1, &index, 2);
    uploadMsg[3] = subindex;

    const std::string & uploadStr = CanUtils::msgToStr(id, COB, sizeof(uploadMsg), uploadMsg);

    if (!send(uploadMsg, sizeof(uploadMsg)))
    {
        CD_ERROR("Unable to send \"%s\" query. %s\n", name.c_str(), uploadStr.c_str());
        return false;
    }

    CD_INFO("Sent \"%s\" query. %s\n", name.c_str(), uploadStr.c_str());

    uint8_t responseMsg[8];
    size_t len;
    bool success = sdoSemaphore->await(responseMsg, &len);
    const std::string & responseStr = CanUtils::msgToStr(id, COB, len, responseMsg);

    if (!success)
    {
        CD_ERROR("Did not receive \"%s\" response. %s\n", name.c_str(), responseStr.c_str());
        return false;
    }

    CD_SUCCESS("Received \"%s\" response. %s\n", name.c_str(), responseStr.c_str());

    std::memcpy(data, responseMsg, len);

    return true;
}

template<typename T>
bool roboticslab::SdoClient::download(const std::string & name, T data, uint16_t index, uint8_t subindex)
{
    const size_t dataSize = sizeof(T);
    const size_t msgSize = dataSize + 4;
    uint8_t downloadMsg[msgSize];

    downloadMsg[0] = 0x23 + ((4 - dataSize) << 2); // client command specifier
    std::memcpy(downloadMsg + 1, &index, 2);
    downloadMsg[3] = subindex;
    std::memcpy(downloadMsg + 4, &data, dataSize);

    const std::string & downloadStr = CanUtils::msgToStr(id, COB, msgSize, downloadMsg);

    if (!send(downloadMsg, msgSize))
    {
        CD_ERROR("Unable to send \"%s\" request. %s\n", name.c_str(), downloadStr.c_str());
        return false;
    }

    CD_INFO("Sent \"%s\" request. %s\n", name.c_str(), downloadStr.c_str());

    uint8_t responseMsg[8];
    size_t len;
    bool success = sdoSemaphore->await(responseMsg, &len);
    const std::string & responseStr = CanUtils::msgToStr(id, COB, msgSize, responseMsg);

    if (!success)
    {
        CD_ERROR("Did not receive \"%s\" ack. %s\n", name.c_str(), responseStr.c_str());
        return false;
    }

    CD_SUCCESS("Received \"%s\" ack. %s\n", name.c_str(), responseStr.c_str());

    return true;
}

#endif // __SDO_CLIENT_INL_HPP__
