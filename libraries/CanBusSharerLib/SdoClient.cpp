// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "SdoClient.hpp"

#include <ColorDebug.h>

#include "CanUtils.hpp"

using namespace roboticslab;

bool SdoClient::send(uint8_t * msg, int len)
{
    return sender->prepareMessage(message_builder(COB + id, len, msg));
}

template<typename T>
bool SdoClient::upload(const std::string & name, T * data, uint16_t index, uint8_t subindex)
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

    SdoSemaphore::sdo_data responseMsg;
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
bool SdoClient::download(const std::string & name, T data, uint16_t index, uint8_t subindex)
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

    SdoSemaphore::sdo_data responseMsg;
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

template bool SdoClient::upload<uint8_t>(const std::string &, uint8_t *, uint16_t, uint8_t);
template bool SdoClient::upload<uint16_t>(const std::string &, uint16_t *, uint16_t, uint8_t);
template bool SdoClient::upload<uint32_t>(const std::string &, uint32_t *, uint16_t, uint8_t);

template bool SdoClient::upload<int8_t>(const std::string &, int8_t *, uint16_t, uint8_t);
template bool SdoClient::upload<int16_t>(const std::string &, int16_t *, uint16_t, uint8_t);
template bool SdoClient::upload<int32_t>(const std::string &, int32_t *, uint16_t, uint8_t);

template bool SdoClient::download<uint8_t>(const std::string &, uint8_t, uint16_t, uint8_t);
template bool SdoClient::download<uint16_t>(const std::string &, uint16_t, uint16_t, uint8_t);
template bool SdoClient::download<uint32_t>(const std::string &, uint32_t, uint16_t, uint8_t);

template bool SdoClient::download<int8_t>(const std::string &, int8_t, uint16_t, uint8_t);
template bool SdoClient::download<int16_t>(const std::string &, int16_t, uint16_t, uint8_t);
template bool SdoClient::download<int32_t>(const std::string &, int32_t, uint16_t, uint8_t);
