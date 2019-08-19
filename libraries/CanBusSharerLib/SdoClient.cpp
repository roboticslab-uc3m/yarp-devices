// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "SdoClient.hpp"

#include <cstring>

#include <sstream>
#include <string>

#include <ColorDebug.h>

#include "CanUtils.hpp"

using namespace roboticslab;

namespace
{
    std::string parseAbortCode(uint32_t code)
    {
        // CiA 301 v4.2.0
        switch (code)
        {
        case 0x05030000:
            return "Toggle bit not alternated";
        case 0x05040000:
            return "SDO protocol timed out";
        case 0x05040001:
            return "Client/server command specifier not valid or unknown";
        case 0x05040002:
            return "Invalid block size (block mode only)";
        case 0x05040003:
            return "Invalid sequence number (block mode only)";
        case 0x05040004:
            return "CRC error (block mode only)";
        case 0x05040005:
            return "Out of memory";
        case 0x06010000:
            return "Unsupported access to an object";
        case 0x06010001:
            return "Attempt to read a write only object";
        case 0x06010002:
            return "Attempt to write a read only object";
        case 0x06020000:
            return "Object does not exist in the object dictionary";
        case 0x06040041:
            return "Object cannot be mapped to the PDO";
        case 0x06040042:
            return "The number and length of the objects to be mapped would exceed PDO length";
        case 0x06040043:
            return "General parameter incompatibility reason";
        case 0x06040047:
            return "General internal incompatibility in the device";
        case 0x06060000:
            return "Access failed due to an hardware error";
        case 0x06070010:
            return "Data type does not match, length of service parameter does not match";
        case 0x06070012:
            return "Data type does not match, length of service parameter too high";
        case 0x06070013:
            return "Data type does not match, length of service parameter too low";
        case 0x06090011:
            return "Sub-index does not exist";
        case 0x06090030:
            return "Invalid value for parameter (download only)";
        case 0x06090031:
            return "Value of parameter written too high (download only)";
        case 0x06090032:
            return "Value of parameter written too low (download only)";
        case 0x06090036:
            return "Maximum value is less than minimum value";
        case 0x060A0023:
            return "Resource not available: SDO connection";
        case 0x08000000:
            return "General error";
        case 0x08000020:
            return "Data cannot be transferred or stored to the application";
        case 0x08000021:
            return "Data cannot be transferred or stored to the application because of local control";
        case 0x08000022:
            return "Data cannot be transferred or stored to the application because of the present device state";
        case 0x08000023:
            return "Object dictionary dynamic generation fails or no object dictionary is present (e.g. object "
                   "dictionary is generated from file and generation fails because of an file error)";
        case 0x08000024:
            return "No data available";
        default:
            return "unknown";
        }
    }
}

bool SdoClient::send(uint8_t * msg, int len)
{
    return sender->prepareMessage(message_builder(COB_D + id, len, msg));
}

template<typename T>
bool SdoClient::upload(const std::string & name, T * data, uint16_t index, uint8_t subindex)
{
    uint8_t uploadMsg[4];

    uploadMsg[0] = 0x40;
    std::memcpy(uploadMsg + 1, &index, 2);
    uploadMsg[3] = subindex;

    const std::string & uploadStr = CanUtils::msgToStr(id, COB_D, sizeof(uploadMsg), uploadMsg);

    if (!send(uploadMsg, sizeof(uploadMsg)))
    {
        CD_ERROR("Unable to send \"%s\" query. %s\n", name.c_str(), uploadStr.c_str());
        return false;
    }

    CD_INFO("Sent \"%s\" query. %s\n", name.c_str(), uploadStr.c_str());

    SdoSemaphore::sdo_data responseMsg;
    size_t len;
    bool success = sdoSemaphore->await(responseMsg, &len);
    const std::string & responseStr = CanUtils::msgToStr(id, COB_U, len, responseMsg);

    if (!success)
    {
        CD_ERROR("Did not receive \"%s\" response. %s\n", name.c_str(), responseStr.c_str());
        return false;
    }

    if (responseMsg[0] == 0x80) // SDO abort transfer (ccs)
    {
        uint32_t code;
        std::memcpy(&code, responseMsg.storage + 4, 4);
        CD_ERROR("SDO transfer abort: %s. %s\n", parseAbortCode(code).c_str(), responseStr.c_str());
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

    const std::string & downloadStr = CanUtils::msgToStr(id, COB_D, msgSize, downloadMsg);

    if (!send(downloadMsg, msgSize))
    {
        CD_ERROR("Unable to send \"%s\" request. %s\n", name.c_str(), downloadStr.c_str());
        return false;
    }

    CD_INFO("Sent \"%s\" request. %s\n", name.c_str(), downloadStr.c_str());

    SdoSemaphore::sdo_data responseMsg;
    size_t len;
    bool success = sdoSemaphore->await(responseMsg, &len);
    const std::string & responseStr = CanUtils::msgToStr(id, COB_U, msgSize, responseMsg);

    if (!success)
    {
        CD_ERROR("Did not receive \"%s\" ack. %s\n", name.c_str(), responseStr.c_str());
        return false;
    }

    if (responseMsg[0] == 0x80) // SDO abort transfer (ccs)
    {
        uint32_t code;
        std::memcpy(&code, responseMsg.storage + 4, 4);
        CD_ERROR("SDO transfer abort: %s. %s\n", parseAbortCode(code).c_str(), responseStr.c_str());
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
