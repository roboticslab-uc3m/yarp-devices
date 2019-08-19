// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "SdoClient.hpp"

#include <cstring>

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

bool SdoClient::send(const uint8_t * msg, int len)
{
    return sender->prepareMessage(message_builder(COB_D + id, len, msg));
}

bool SdoClient::expeditedUpload(const std::string & name, void * data, size_t size, uint16_t index, uint8_t subindex)
{
    uint8_t uploadMsg[4];

    uploadMsg[0] = 0x40; // client command specifier
    std::memcpy(uploadMsg + 1, &index, 2);
    uploadMsg[3] = subindex;

    return performTransfer(name, uploadMsg, sizeof(uploadMsg), data, size);
}

bool SdoClient::expeditedDownload(const std::string & name, const void * data, size_t size, uint16_t index, uint8_t subindex)
{
    const size_t msgSize = size + 4;
    uint8_t downloadMsg[msgSize];

    downloadMsg[0] = 0x23 + ((4 - size) << 2); // client command specifier
    std::memcpy(downloadMsg + 1, &index, 2);
    downloadMsg[3] = subindex;
    std::memcpy(downloadMsg + 4, &data, size);

    return performTransfer(name, downloadMsg, msgSize);
}

bool SdoClient::performTransfer(const std::string & name, const uint8_t * req, size_t reqSize, void * resp, size_t respSize)
{
    const std::string & reqStr = CanUtils::msgToStr(id, COB_D, reqSize, req);

    if (!send(req, reqSize))
    {
        CD_ERROR("Unable to send \"%s\" request. %s\n", name.c_str(), reqStr.c_str());
        return false;
    }

    CD_INFO("Sent \"%s\" request. %s\n", name.c_str(), reqStr.c_str());

    SdoSemaphore::sdo_data sdoResponse;
    size_t len;
    bool success = sdoSemaphore->await(sdoResponse, &len);
    const std::string & respStr = CanUtils::msgToStr(id, COB_U, len, sdoResponse);

    if (!success)
    {
        CD_ERROR("Did not receive \"%s\" ack. %s\n", name.c_str(), respStr.c_str());
        return false;
    }

    if (sdoResponse[0] == 0x80) // SDO abort transfer (ccs)
    {
        uint32_t code;
        std::memcpy(&code, sdoResponse.storage + 4, sizeof(code));
        CD_ERROR("SDO transfer abort: %s. %s\n", parseAbortCode(code).c_str(), respStr.c_str());
        return false;
    }

    if (resp != 0)
    {
        if (respSize != len - 4)
        {
            CD_ERROR("Expected response size %d, got %d.\n", respSize, len - 4);
            return false;
        }

        std::memcpy(resp, sdoResponse.storage + 4, respSize);
    }

    CD_SUCCESS("Received \"%s\" ack. %s\n", name.c_str(), respStr.c_str());

    return true;
}
