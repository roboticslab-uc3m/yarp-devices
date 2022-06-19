// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusSocket.hpp"

#include <linux/can/error.h>

#include <sys/select.h>
#include <sys/time.h>

#include <cstring>
#include <cerrno>
#include <cassert>

#include <stdexcept>
#include <string>
#include <vector>

#include <yarp/os/Log.h>

#include "LogComponent.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

namespace
{
    void setTimeval(int timeMs, struct timeval * tv)
    {
        tv->tv_sec = timeMs / 1000;
        tv->tv_usec = (timeMs % 1000) * 1000;
    }
}

// -----------------------------------------------------------------------------

bool CanBusSocket::waitUntilTimeout(io_operation op, bool * bufferReady)
{
    fd_set fds;

    FD_ZERO(&fds);
    FD_SET(s, &fds);

    struct timeval tv;

    //-- select() returns the number of ready descriptors, 0 for timeout, -1 for errors.
    int ret;

    switch (op)
    {
    case READ:
        setTimeval(rxTimeoutMs, &tv);
        ret = ::select(s + 1, &fds, nullptr, nullptr, &tv);
        break;
    case WRITE:
        setTimeval(txTimeoutMs, &tv);
        ret = ::select(s + 1, nullptr, &fds, nullptr, &tv);
        break;
    default:
        yCIError(SCK, id(), "Unhandled IO operation on select()");
        return false;
    }

    if (ret < 0)
    {
        yCIError(SCK, id(), "select() error: %s", std::strerror(errno));
        return false;
    }
    else if (ret == 0)
    {
        *bufferReady = false;
    }
    else
    {
        assert(FD_ISSET(s, &fds));
        *bufferReady = true;
    }

    return true;
}

// -----------------------------------------------------------------------------

void CanBusSocket::interpretErrorFrame(const struct can_frame * msg)
{
    if (msg->can_id & CAN_ERR_TX_TIMEOUT)
    {
        yCIWarning(SCK, id(), "Error: TX timeout");
    }

    if (msg->can_id & CAN_ERR_LOSTARB)
    {
        if (msg->data[0] == CAN_ERR_LOSTARB_UNSPEC)
        {
            yCIWarning(SCK, id(), "Lost arbitration: unspecified");
        }
        else
        {
            yCIWarning(SCK, id(), "Lost arbitration: bit %d", msg->data[0]);
        }
    }

    if (msg->can_id & CAN_ERR_CRTL)
    {
        if (msg->data[1] == CAN_ERR_CRTL_UNSPEC)
        {
            yCIWarning(SCK, id(), "Controller: unspecified");
        }
        else
        {
            if (msg->data[1] & CAN_ERR_CRTL_RX_OVERFLOW)
            {
                yCIWarning(SCK, id(), "Controller: RX buffer overflow");
            }

            if (msg->data[1] & CAN_ERR_CRTL_TX_OVERFLOW)
            {
                yCIWarning(SCK, id(), "Controller: TX buffer overflow");
            }

            if (msg->data[1] & CAN_ERR_CRTL_RX_WARNING)
            {
                yCIWarning(SCK, id(), "Controller: reached warning level for RX errors");
            }

            if (msg->data[1] & CAN_ERR_CRTL_TX_WARNING)
            {
                yCIWarning(SCK, id(), "Controller: reached warning level for TX errors");
            }

            if (msg->data[1] & CAN_ERR_CRTL_RX_PASSIVE)
            {
                yCIWarning(SCK, id(), "Controller: reached error passive status RX");
            }

            if (msg->data[1] & CAN_ERR_CRTL_TX_PASSIVE)
            {
                yCIWarning(SCK, id(), "Controller: reached error passive status TX");
            }

            if (msg->data[1] & CAN_ERR_CRTL_ACTIVE)
            {
                yCIWarning(SCK, id(), "Controller: recovered to error active state");
            }
        }
    }

    if (msg->can_id & CAN_ERR_PROT)
    {
        std::vector<std::string> types;
        std::string location;

        if (msg->data[2] == CAN_ERR_PROT_UNSPEC)
        {
            types = {"unspecified type"};
        }
        else
        {
            if (msg->data[2] & CAN_ERR_PROT_BIT)
            {
                types.emplace_back("single bit error");
            }

            if (msg->data[2] & CAN_ERR_PROT_FORM)
            {
                types.emplace_back("frame format error");
            }

            if (msg->data[2] & CAN_ERR_PROT_STUFF)
            {
                types.emplace_back("bit stuffing error");
            }

            if (msg->data[2] & CAN_ERR_PROT_BIT0)
            {
                types.emplace_back("unable to send dominant bit");
            }

            if (msg->data[2] & CAN_ERR_PROT_BIT1)
            {
                types.emplace_back("unable to send recessive bit");
            }

            if (msg->data[2] & CAN_ERR_PROT_OVERLOAD)
            {
                types.emplace_back("bus overload");
            }

            if (msg->data[2] & CAN_ERR_PROT_ACTIVE)
            {
                types.emplace_back("active error announcement");
            }

            if (msg->data[2] & CAN_ERR_PROT_TX)
            {
                types.emplace_back("error occurred on transmission");
            }
        }

        switch (msg->data[3])
        {
        case CAN_ERR_PROT_LOC_UNSPEC:
            location = "unspecified location";
            break;
        case CAN_ERR_PROT_LOC_SOF:
            location = "start of frame";
            break;
        case CAN_ERR_PROT_LOC_ID28_21:
            location = "ID bits 28-21 (SFF: 10-3)";
            break;
        case CAN_ERR_PROT_LOC_ID20_18:
            location = "ID bits 20-18 (SFF: 2-0)";
            break;
        case CAN_ERR_PROT_LOC_SRTR:
            location = "substitute RTR (SFF: RTR)";
            break;
        case CAN_ERR_PROT_LOC_IDE:
            location = "identifier extension";
            break;
        case CAN_ERR_PROT_LOC_ID17_13:
            location = "ID bits 17-13";
            break;
        case CAN_ERR_PROT_LOC_ID12_05:
            location = "ID bits 12-5";
            break;
        case CAN_ERR_PROT_LOC_ID04_00:
            location = "ID bits 4-0";
            break;
        case CAN_ERR_PROT_LOC_RTR:
            location = "RTR";
            break;
        case CAN_ERR_PROT_LOC_RES1:
            location = "reserved bit 1";
            break;
        case CAN_ERR_PROT_LOC_RES0:
            location = "reserved bit 0";
            break;
        case CAN_ERR_PROT_LOC_DLC:
            location = "data length code";
            break;
        case CAN_ERR_PROT_LOC_DATA:
            location = "data section";
            break;
        case CAN_ERR_PROT_LOC_CRC_SEQ:
            location = "CRC sequence";
            break;
        case CAN_ERR_PROT_LOC_CRC_DEL:
            location = "CRC delimiter";
            break;
        case CAN_ERR_PROT_LOC_ACK:
            location = "ACK slot";
            break;
        case CAN_ERR_PROT_LOC_ACK_DEL:
            location = "ACK delimiter";
            break;
        case CAN_ERR_PROT_LOC_EOF:
            location = "end of frame";
            break;
        case CAN_ERR_PROT_LOC_INTERM:
            location = "intermission";
            break;
        default:
            location = "unknown location";
            break;
        }

        for (const auto & type : types)
        {
            yCIWarning(SCK, id(), "Protocol violation: %s at %s", type.c_str(), location.c_str());
        }
    }

    if (msg->can_id & CAN_ERR_TRX)
    {
        if (msg->data[4] == CAN_ERR_TRX_UNSPEC)
        {
            yCIWarning(SCK, id(), "Transceiver status: unspecified");
        }
        else
        {
            switch (msg->data[4] & 0x07)
            {
            case CAN_ERR_TRX_CANH_NO_WIRE:
                yCIWarning(SCK, id(), "Transceiver status (CAN-H): no wire");
                break;
            case CAN_ERR_TRX_CANH_SHORT_TO_BAT:
                yCIWarning(SCK, id(), "Transceiver status (CAN-H): short to BAT");
                break;
            case CAN_ERR_TRX_CANH_SHORT_TO_VCC:
                yCIWarning(SCK, id(), "Transceiver status (CAN-H): short to VCC");
                break;
            case CAN_ERR_TRX_CANH_SHORT_TO_GND:
                yCIWarning(SCK, id(), "Transceiver status (CAN-H): short to GND");
                break;
            }

            switch (msg->data[4] & 0x70)
            {
            case CAN_ERR_TRX_CANL_NO_WIRE:
                yCIWarning(SCK, id(), "Transceiver status (CAN-L): no wire");
                break;
            case CAN_ERR_TRX_CANL_SHORT_TO_BAT:
                yCIWarning(SCK, id(), "Transceiver status (CAN-L): short to BAT");
                break;
            case CAN_ERR_TRX_CANL_SHORT_TO_VCC:
                yCIWarning(SCK, id(), "Transceiver status (CAN-L): short to VCC");
                break;
            case CAN_ERR_TRX_CANL_SHORT_TO_GND:
                yCIWarning(SCK, id(), "Transceiver status (CAN-L): short to GND");
                break;
            }

            if ((msg->data[4] & 0x80) == CAN_ERR_TRX_CANL_SHORT_TO_CANH)
            {
                yCIWarning(SCK, id(), "Transceiver status: short between CAN-L and CAN-H");
            }
        }
    }

    if (msg->can_id & CAN_ERR_ACK)
    {
        yCIWarning(SCK, id(), "Received no ACK on transmission");
    }

    if (msg->can_id & CAN_ERR_BUSOFF)
    {
        yCIWarning(SCK, id(), "Bus off");
    }

    if (msg->can_id & CAN_ERR_BUSERROR)
    {
        yCIWarning(SCK, id(), "Bus error");
    }

    if (msg->can_id & CAN_ERR_RESTARTED)
    {
        yCIWarning(SCK, id(), "Controller restarted");
    }
}

// -----------------------------------------------------------------------------
