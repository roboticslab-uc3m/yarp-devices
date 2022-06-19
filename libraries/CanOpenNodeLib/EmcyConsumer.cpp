// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "EmcyConsumer.hpp"

#include <cstring>

using namespace roboticslab;

std::string EmcyCodeRegistry::codeToMessage(std::uint16_t code)
{
    switch (code)
    {
    case 0x0000:
        return "Error reset or no error";
    case 0x1000:
        return "Generic error";
    case 0x2000:
        return "Current - generic error";
    case 0x2100:
        return "Current, CANopen device input side – generic";
    case 0x2200:
        return "Current inside the CANopen device – generic";
    case 0x2300:
        return "Current, CANopen device output side – generic";
    case 0x3000:
        return "Voltage – generic error";
    case 0x3100:
        return "Mains voltage – generic";
    case 0x3200:
        return "Voltage inside the CANopen device – generic";
    case 0x3300:
        return "Output voltage – generic";
    case 0x4000:
        return "Temperature – generic error";
    case 0x4100:
        return "Ambient temperature – generic";
    case 0x4200:
        return "Device temperature – generic";
    case 0x5000:
        return "CANopen device hardware – generic error";
    case 0x6000:
        return "CANopen device software – generic error";
    case 0x6100:
        return "Internal software – generic";
    case 0x6200:
        return "User software – generic";
    case 0x6300:
        return "Data set – generic";
    case 0x7000:
        return "Additional modules – generic error";
    case 0x8000:
        return "Monitoring – generic error";
    case 0x8100:
        return "Communication – generic";
    case 0x8110:
        return "CAN overrun (objects lost)";
    case 0x8120:
        return "CAN in error passive mode";
    case 0x8130:
        return "Life guard error or heartbeat error";
    case 0x8140:
        return "recovered from bus off";
    case 0x8150:
        return "CAN-ID collision";
    case 0x8200:
        return "Protocol error - generic";
    case 0x8210:
        return "PDO not processed due to length error";
    case 0x8220:
        return "PDO length exceeded";
    case 0x8230:
        return "DAM MPDO not processed, destination object not available";
    case 0x8240:
        return "Unexpected SYNC data length";
    case 0x8250:
        return "RPDO timeout";
    case 0x9000:
        return "External error – generic error";
    case 0xF000:
        return "Additional functions – generic error";
    case 0xFF00:
        return "Device specific – generic error";
    default:
        return "unknown";
    }
}

bool EmcyConsumer::accept(const std::uint8_t * data)
{
    if (!callback)
    {
        return false;
    }

    std::uint16_t code;
    std::uint8_t reg;
    std::uint8_t msef[5];

    std::memcpy(&code, data, 2);
    std::memcpy(&reg, data + 2, 1);
    std::memcpy(msef, data + 3, 5);

    code_t codeToMsg = {code, codeRegistry->codeToMessage(code)};

    callback(codeToMsg, reg, msef);
    return true;
}
