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
        yCError(SCK, "Unhandled IO operation on select()");
        return false;
    }

    if (ret < 0)
    {
        yCError(SCK, "select() error: %s", std::strerror(errno));
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

void CanBusSocket::interpretErrorFrame(const struct can_frame * msg) const
{
    if (msg->can_id & CAN_ERR_TX_TIMEOUT)
    {
        yCWarning(SCK, "Error: TX timeout (%s)", iface.c_str());
    }

    if (msg->can_id & CAN_ERR_LOSTARB)
    {
        if (msg->data[0] == CAN_ERR_LOSTARB_UNSPEC)
        {
            yCWarning(SCK, "Lost arbitration: unspecified (%s)", iface.c_str());
        }
        else
        {
            yCWarning(SCK, "Lost arbitration: bit %d (%s)", msg->data[0], iface.c_str());
        }
    }

    if (msg->can_id & CAN_ERR_CRTL)
    {
        if (msg->data[1] == CAN_ERR_CRTL_UNSPEC)
        {
            yCWarning(SCK, "Controller: unspecified (%s)", iface.c_str());
        }
        else
        {
            if (msg->data[1] & CAN_ERR_CRTL_RX_OVERFLOW)
            {
                yCWarning(SCK, "Controller: RX buffer overflow (%s)", iface.c_str());
            }

            if (msg->data[1] & CAN_ERR_CRTL_TX_OVERFLOW)
            {
                yCWarning(SCK, "Controller: TX buffer overflow (%s)", iface.c_str());
            }

            if (msg->data[1] & CAN_ERR_CRTL_RX_WARNING)
            {
                yCWarning(SCK, "Controller: reached warning level for RX errors (%s)", iface.c_str());
            }

            if (msg->data[1] & CAN_ERR_CRTL_TX_WARNING)
            {
                yCWarning(SCK, "Controller: reached warning level for TX errors (%s)", iface.c_str());
            }

            if (msg->data[1] & CAN_ERR_CRTL_RX_PASSIVE)
            {
                yCWarning(SCK, "Controller: reached error passive status RX (%s)", iface.c_str());
            }

            if (msg->data[1] & CAN_ERR_CRTL_TX_PASSIVE)
            {
                yCWarning(SCK, "Controller: reached error passive status TX (%s)", iface.c_str());
            }

            if (msg->data[1] & CAN_ERR_CRTL_ACTIVE)
            {
                yCWarning(SCK, "Controller: recovered to error active state (%s)", iface.c_str());
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
            yCWarning(SCK, "Protocol violation: %s at %s (%s)", type.c_str(), location.c_str(), iface.c_str());
        }
    }

    if (msg->can_id & CAN_ERR_TRX)
    {
        if (msg->data[4] == CAN_ERR_TRX_UNSPEC)
        {
            yCWarning(SCK, "Transceiver status: unspecified (%s)", iface.c_str());
        }
        else
        {
            switch (msg->data[4] & 0x07)
            {
            case CAN_ERR_TRX_CANH_NO_WIRE:
                yCWarning(SCK, "Transceiver status (CAN-H): no wire (%s)", iface.c_str());
                break;
            case CAN_ERR_TRX_CANH_SHORT_TO_BAT:
                yCWarning(SCK, "Transceiver status (CAN-H): short to BAT (%s)", iface.c_str());
                break;
            case CAN_ERR_TRX_CANH_SHORT_TO_VCC:
                yCWarning(SCK, "Transceiver status (CAN-H): short to VCC (%s)", iface.c_str());
                break;
            case CAN_ERR_TRX_CANH_SHORT_TO_GND:
                yCWarning(SCK, "Transceiver status (CAN-H): short to GND (%s)", iface.c_str());
                break;
            }

            switch (msg->data[4] & 0x70)
            {
            case CAN_ERR_TRX_CANL_NO_WIRE:
                yCWarning(SCK, "Transceiver status (CAN-L): no wire (%s)", iface.c_str());
                break;
            case CAN_ERR_TRX_CANL_SHORT_TO_BAT:
                yCWarning(SCK, "Transceiver status (CAN-L): short to BAT (%s)", iface.c_str());
                break;
            case CAN_ERR_TRX_CANL_SHORT_TO_VCC:
                yCWarning(SCK, "Transceiver status (CAN-L): short to VCC (%s)", iface.c_str());
                break;
            case CAN_ERR_TRX_CANL_SHORT_TO_GND:
                yCWarning(SCK, "Transceiver status (CAN-L): short to GND (%s)", iface.c_str());
                break;
            }

            if ((msg->data[4] & 0x80) == CAN_ERR_TRX_CANL_SHORT_TO_CANH)
            {
                yCWarning(SCK, "Transceiver status: short between CAN-L and CAN-H (%s)", iface.c_str());
            }
        }
    }

    if (msg->can_id & CAN_ERR_ACK)
    {
        yCWarning(SCK, "Received no ACK on transmission (%s)", iface.c_str());
    }

    if (msg->can_id & CAN_ERR_BUSOFF)
    {
        yCWarning(SCK, "Bus off (%s)", iface.c_str());
    }

    if (msg->can_id & CAN_ERR_BUSERROR)
    {
        yCWarning(SCK, "Bus error (%s)", iface.c_str());
    }

    if (msg->can_id & CAN_ERR_RESTARTED)
    {
        yCWarning(SCK, "Controller restarted (%s)", iface.c_str());
    }
}

// -----------------------------------------------------------------------------
