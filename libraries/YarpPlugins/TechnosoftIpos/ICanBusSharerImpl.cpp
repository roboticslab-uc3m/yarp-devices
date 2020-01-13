// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TechnosoftIpos.hpp"

#include <cctype> // std::isspace

#include <algorithm> // std::find_if

#include <yarp/os/Time.h>

#include <ColorDebug.h>

#include "CanUtils.hpp"

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

unsigned int TechnosoftIpos::getId()
{
    return can->getId();
}

// -----------------------------------------------------------------------------

std::vector<unsigned int> TechnosoftIpos::getAdditionalIds()
{
    if (iExternalEncoderCanBusSharer)
    {
        return {iExternalEncoderCanBusSharer->getId()};
    }

    return {};
}

// -----------------------------------------------------------------------------

bool TechnosoftIpos::registerSender(CanSenderDelegate * sender)
{
    can->configureSender(sender);
    return iExternalEncoderCanBusSharer && iExternalEncoderCanBusSharer->registerSender(sender);
}

// -----------------------------------------------------------------------------

bool TechnosoftIpos::initialize()
{
    if (!can->sdo()->ping())
    {
        return false;
    }

    if (!vars.configuredOnce)
    {
        // retrieve static drive info
        vars.configuredOnce = can->sdo()->upload<std::uint32_t>("Device type",
                [](std::uint32_t data)
                {
                    CD_INFO("CiA standard: %d.\n", data & 0xFFFF);
                },
                0x1000)
            && can->sdo()->upload<std::uint32_t>("Supported drive modes",
                [this](std::uint32_t data)
                {
                    interpretSupportedDriveModes(data);
                },
                0x6502)
            && can->sdo()->upload("Manufacturer software version",
                [](const std::string & firmware)
                {
                    std::string s(firmware);
                    CD_INFO("Firmware version: %s.\n", rtrim(s).c_str());
                },
                0x100A)
            && can->sdo()->upload<std::uint32_t>("Identity Object: Product Code",
                [](std::uint32_t data)
                {
                    CD_INFO("Product code: P%03d.%03d.E%03d.\n", data / 1000000, (data / 1000) % 1000, data % 1000);
                },
                0x1018, 0x02)
            && can->sdo()->upload<std::uint32_t>("Identity Object: Serial number",
                [](std::uint32_t data)
                {
                    CD_INFO("Serial number: %c%c%02x%02x.\n", getByte(data, 3), getByte(data, 2), getByte(data, 1), getByte(data, 0));
                },
                0x1018, 0x04);
    }

    double extEnc;

    return vars.configuredOnce
        && (!iExternalEncoderCanBusSharer || iExternalEncoderCanBusSharer->initialize())
        && setLimitsRaw(0, vars.min, vars.max)
        && setRefSpeedRaw(0, vars.refSpeed)
        && setRefAccelerationRaw(0, vars.refAcceleration)
        // synchronize absolute (master) and relative (slave) encoders
        && (!iEncodersTimedRawExternal || (iEncodersTimedRawExternal->getEncodersRaw(&extEnc) && setEncoderRaw(0, extEnc)))
        && can->tpdo1()->configure(vars.tpdo1Conf)
        && can->tpdo2()->configure(vars.tpdo2Conf)
        && can->tpdo3()->configure(vars.tpdo3Conf)
        && can->sdo()->download<std::uint16_t>("Producer Heartbeat Time", vars.heartbeatPeriod, 0x1017)
        && (vars.lastHeartbeat = yarp::os::Time::now(), true)
        && (vars.actualControlMode = VOCAB_CM_CONFIGURED, true)
        && can->nmt()->issueServiceCommand(NmtService::START_REMOTE_NODE)
        && (can->driveStatus()->getCurrentState() != DriveState::NOT_READY_TO_SWITCH_ON
            || can->driveStatus()->awaitState(DriveState::SWITCH_ON_DISABLED))
        && can->driveStatus()->requestState(DriveState::SWITCHED_ON)
        && (vars.actualControlMode = VOCAB_CM_IDLE, true)
        && setControlModeRaw(0, vars.initialMode);
}

// -----------------------------------------------------------------------------

bool TechnosoftIpos::finalize()
{
    bool ok = true;

    if (vars.actualControlMode != VOCAB_CM_NOT_CONFIGURED)
    {
        if (can->driveStatus()->getCurrentState() == DriveState::OPERATION_ENABLED)
        {
            can->driveStatus()->requestTransition(DriveTransition::QUICK_STOP);
        }

        if (!can->driveStatus()->requestState(DriveState::SWITCH_ON_DISABLED))
        {
            CD_WARNING("SWITCH_ON_DISABLED transition failed.\n");
            ok = false;
        }

        vars.actualControlMode = VOCAB_CM_CONFIGURED;
    }

    if (!can->nmt()->issueServiceCommand(NmtService::RESET_NODE))
    {
        CD_WARNING("Reset node NMT service failed.\n");
        ok = false;
    }

    if (iExternalEncoderCanBusSharer)
    {
        ok = ok && iExternalEncoderCanBusSharer->finalize();
    }

    vars.actualControlMode = VOCAB_CM_NOT_CONFIGURED;
    return ok;
}

// -----------------------------------------------------------------------------

bool TechnosoftIpos::interpretMessage(const yarp::dev::CanMessage & message)
{
    if (iExternalEncoderCanBusSharer && iExternalEncoderCanBusSharer->getId() == (message.getId() & 0x7F))
    {
        return iExternalEncoderCanBusSharer->interpretMessage(message);
    }

    if (!can->consumeMessage(message.getId(), message.getData(), message.getLen()))
    {
        CD_WARNING("Unknown message: %s\n", CanUtils::msgToStr(message.getId(), message.getLen(), message.getData()).c_str());
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------------
