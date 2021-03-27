// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TechnosoftIpos.hpp"

#include <cctype> // std::isspace

#include <algorithm> // std::find_if

#include <yarp/os/Time.h>
#include <yarp/os/Vocab.h>

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
                [](auto data)
                { CD_INFO("CiA standard: %d.\n", data & 0xFFFF); },
                0x1000)
            && can->sdo()->upload<std::uint32_t>("Supported drive modes",
                [this](auto data)
                { interpretSupportedDriveModes(data); },
                0x6502)
            && can->sdo()->upload("Manufacturer software version",
                [](const auto & data)
                { CD_INFO("Firmware version: %s.\n", rtrim(data).c_str()); },
                0x100A)
            && can->sdo()->upload<std::uint32_t>("Identity Object: Product Code",
                [](auto data)
                { CD_INFO("Product code: P%03d.%03d.E%03d.\n", data / 1000000, (data / 1000) % 1000, data % 1000); },
                0x1018, 0x02)
            && can->sdo()->upload<std::uint32_t>("Identity Object: Serial number",
                [](auto data)
                { CD_INFO("Serial number: %c%c%02x%02x.\n", getByte(data, 3), getByte(data, 2), getByte(data, 1), getByte(data, 0)); },
                0x1018, 0x04);
    }

    double extEnc;

    if (!vars.configuredOnce
        || (iExternalEncoderCanBusSharer && !iExternalEncoderCanBusSharer->initialize())
        || !setLimitsRaw(0, vars.min, vars.max)
        || !setRefSpeedRaw(0, vars.refSpeed)
        || !setRefAccelerationRaw(0, vars.refAcceleration)
        // synchronize absolute (master) and relative (slave) encoders
        || (iEncodersTimedRawExternal && (!iEncodersTimedRawExternal->getEncodersRaw(&extEnc) || !setEncoderRaw(0, extEnc)))
        || !can->tpdo1()->configure(vars.tpdo1Conf)
        || !can->tpdo2()->configure(vars.tpdo2Conf)
        || !can->tpdo3()->configure(vars.tpdo3Conf)
        || (vars.heartbeatPeriod != 0.0
                && !can->sdo()->download<std::uint16_t>("Producer Heartbeat Time", vars.heartbeatPeriod * 1000, 0x1017))
        || !can->nmt()->issueServiceCommand(NmtService::START_REMOTE_NODE)
        || (can->driveStatus()->getCurrentState() == DriveState::NOT_READY_TO_SWITCH_ON
                && !can->driveStatus()->awaitState(DriveState::SWITCH_ON_DISABLED)))
    {
        CD_ERROR("Initial SDO configuration and/or node start failed (canId: %d).\n", can->getId());
        return false;
    }

    vars.lastHeartbeat = yarp::os::Time::now();
    vars.actualControlMode = VOCAB_CM_CONFIGURED;

    if (!can->driveStatus()->requestState(DriveState::SWITCHED_ON)
            || !vars.awaitControlMode(VOCAB_CM_IDLE)
            || !setControlModeRaw(0, vars.initialMode))
    {
        CD_WARNING("Initial drive state transitions failed (canId: %d).\n", can->getId());
    }

    return true;
}

// -----------------------------------------------------------------------------

bool TechnosoftIpos::finalize()
{
    if (monitorThread && monitorThread->isRunning())
    {
        monitorThread->stop();
    }

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

bool TechnosoftIpos::notifyMessage(const can_message & message)
{
    if (iExternalEncoderCanBusSharer && iExternalEncoderCanBusSharer->getId() == (message.id & 0x7F))
    {
        return iExternalEncoderCanBusSharer->notifyMessage(message);
    }

    if (!can->notifyMessage(message))
    {
        CD_WARNING("Unknown message: %s\n", CanUtils::msgToStr(message).c_str());
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------------

bool TechnosoftIpos::synchronize()
{
    if (!vars.enableSync)
    {
        return true;
    }

    switch (vars.actualControlMode.load())
    {
    case VOCAB_CM_VELOCITY:
    {
        if (vars.enableCsv)
        {
            double value = vars.synchronousCommandTarget * vars.syncPeriod;
            std::int32_t data = vars.degreesToInternalUnits(value);
            return can->rpdo3()->write(data);
        }
        else
        {
            double value = vars.degreesToInternalUnits(vars.synchronousCommandTarget, 1);

            std::int16_t dataInt;
            std::uint16_t dataFrac;
            CanUtils::encodeFixedPoint(value, &dataInt, &dataFrac);

            std::int32_t data = (dataInt << 16) + dataFrac;
            return can->rpdo3()->write(data);
        }
    }
    case VOCAB_CM_TORQUE:
    {
        double curr = vars.torqueToCurrent(vars.synchronousCommandTarget);
        std::int32_t data = vars.currentToInternalUnits(curr) << 16;
        return can->rpdo3()->write(data);
    }
    case VOCAB_CM_CURRENT:
    {
        std::int32_t data = vars.currentToInternalUnits(vars.synchronousCommandTarget) << 16;
        return can->rpdo3()->write(data);
    }
    case VOCAB_CM_POSITION_DIRECT:
    {
        double value = vars.clipSyncPositionTarget();
        std::int32_t data = vars.degreesToInternalUnits(value);
        return can->rpdo3()->write(data);
    }
    default:
        return true;
    }
}

// -----------------------------------------------------------------------------
