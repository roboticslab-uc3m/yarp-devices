// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TechnosoftIposBase.hpp"

#include <cctype> // std::isspace

#include <algorithm> // std::find_if

#include <yarp/os/LogStream.h>
#include <yarp/os/SystemClock.h>
#include <yarp/os/Vocab.h>

#include "CanUtils.hpp"
#include "LogComponent.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

namespace
{
    inline std::uint8_t getByte(std::uint32_t number, int n)
    {
        // https://stackoverflow.com/a/7787433
        return (number >> (8 * n)) & 0xFF;
    }

    // https://stackoverflow.com/a/217605
    inline std::string rtrim(const std::string & orig)
    {
        std::string s(orig);
        s.erase(std::find_if(s.rbegin(), s.rend(), [](int ch) { return !std::isspace(ch); }).base(), s.end());
        return s;
    }
}

// -----------------------------------------------------------------------------

unsigned int TechnosoftIposBase::getId()
{
    return can->getId();
}

// -----------------------------------------------------------------------------

std::vector<unsigned int> TechnosoftIposBase::getAdditionalIds()
{
    if (iExternalEncoderCanBusSharer)
    {
        return {iExternalEncoderCanBusSharer->getId()};
    }

    return {};
}

// -----------------------------------------------------------------------------

bool TechnosoftIposBase::registerSender(CanSenderDelegate * sender)
{
    can->configureSender(sender);
    return iExternalEncoderCanBusSharer && iExternalEncoderCanBusSharer->registerSender(sender);
}

// -----------------------------------------------------------------------------

bool TechnosoftIposBase::initialize()
{
    if (!can->sdo()->ping())
    {
        return false;
    }

    if (!configuredOnce)
    {
        // retrieve static drive info
        configuredOnce = can->sdo()->upload("Manufacturer software version",
                [this](const auto & data)
                {
                    yCIInfo(IPOS, id()) << "Firmware version:" << rtrim(data);
                },
                0x100A)
            && can->sdo()->upload<std::uint32_t>("Identity Object: Product Code",
                [this](auto data)
                {
                    yCIInfo(IPOS, id(), "Product code: P%03d.%03d.E%03d", data / 1000000, (data / 1000) % 1000, data % 1000);
                },
                0x1018, 0x02)
            && can->sdo()->upload<std::uint32_t>("Identity Object: Serial number",
                [this](auto data)
                {
                    yCIInfo(IPOS, id(), "Serial number: %c%c%02x%02x", getByte(data, 3), getByte(data, 2), getByte(data, 1), getByte(data, 0));
                },
                0x1018, 0x04);
    }

    double extEnc;

    if (!configuredOnce
        || (iExternalEncoderCanBusSharer && !iExternalEncoderCanBusSharer->initialize())
        || !setLimitsRaw(0, min, max)
        || !setRefSpeedRaw(0, refSpeed)
        || !setRefAccelerationRaw(0, refAcceleration)
        // synchronize absolute (master) and relative (slave) encoders
        || (iEncodersTimedRawExternal && (!iEncodersTimedRawExternal->getEncodersRaw(&extEnc) || !setEncoderRaw(0, extEnc)))
        || !can->tpdo1()->configure(tpdo1Conf)
        || !can->tpdo2()->configure(tpdo2Conf)
        || !can->tpdo3()->configure(tpdo3Conf)
        || (heartbeatPeriod != 0.0
                && !can->sdo()->download<std::uint16_t>("Producer Heartbeat Time", heartbeatPeriod * 1000, 0x1017))
        || !can->nmt()->issueServiceCommand(NmtService::START_REMOTE_NODE)
        || (can->driveStatus()->getCurrentState() == DriveState::NOT_READY_TO_SWITCH_ON
                && !can->driveStatus()->awaitState(DriveState::SWITCH_ON_DISABLED)))
    {
        yCIError(IPOS, id()) << "Initial SDO configuration and/or node start failed";
        return false;
    }

    lastHeartbeat = yarp::os::SystemClock::nowSystem();
    actualControlMode = VOCAB_CM_CONFIGURED;

    if (!can->driveStatus()->requestState(DriveState::SWITCHED_ON)
            || !awaitControlMode(VOCAB_CM_IDLE)
            || !setControlModeRaw(0, initialControlMode))
    {
        yCIWarning(IPOS, id()) << "Initial drive state transitions failed";
    }

    return true;
}

// -----------------------------------------------------------------------------

bool TechnosoftIposBase::finalize()
{
    if (monitorThread && monitorThread->isRunning())
    {
        monitorThread->stop();
    }

    bool ok = true;

    if (actualControlMode != VOCAB_CM_NOT_CONFIGURED)
    {
        if (can->driveStatus()->getCurrentState() == DriveState::OPERATION_ENABLED)
        {
            can->driveStatus()->requestTransition(DriveTransition::QUICK_STOP);
        }

        if (!can->driveStatus()->requestState(DriveState::SWITCH_ON_DISABLED))
        {
            yCIWarning(IPOS, id()) << "SWITCH_ON_DISABLED transition failed";
            ok = false;
        }

        actualControlMode = VOCAB_CM_CONFIGURED;
    }

    if (!can->nmt()->issueServiceCommand(NmtService::RESET_NODE))
    {
        yCIWarning(IPOS, id()) << "Reset node NMT service failed";
        ok = false;
    }

    if (iExternalEncoderCanBusSharer)
    {
        ok = ok && iExternalEncoderCanBusSharer->finalize();
    }

    actualControlMode = VOCAB_CM_NOT_CONFIGURED;
    return ok;
}

// -----------------------------------------------------------------------------

bool TechnosoftIposBase::notifyMessage(const can_message & message)
{
    if (iExternalEncoderCanBusSharer && iExternalEncoderCanBusSharer->getId() == (message.id & 0x7F))
    {
        return iExternalEncoderCanBusSharer->notifyMessage(message);
    }

    if (!can->notifyMessage(message))
    {
        yCIWarning(IPOS, id(), "Unknown message: %s", CanUtils::msgToStr(message).c_str());
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------------
