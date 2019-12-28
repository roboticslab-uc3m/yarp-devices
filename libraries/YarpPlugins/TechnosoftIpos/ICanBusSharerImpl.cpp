// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TechnosoftIpos.hpp"

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
    double extEnc;

    return (!iExternalEncoderCanBusSharer || iExternalEncoderCanBusSharer->initialize())
        && can->sdo()->upload<std::uint32_t>("Device type",
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
                CD_INFO("Firmware version: %s.\n", firmware.c_str());
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
            0x1018, 0x04)
        && setLimitsRaw(0, vars.min, vars.max)
        && setRefSpeedRaw(0, vars.refSpeed)
        && setRefAccelerationRaw(0, vars.refAcceleration)
        // synchronize absolute (master) and relative (slave) encoders
        && (!iEncodersTimedRawExternal || (iEncodersTimedRawExternal->getEncodersRaw(&extEnc) && setEncoderRaw(0, extEnc)))
        && can->tpdo1()->configure(vars.tpdo1Conf)
        && can->tpdo2()->configure(vars.tpdo2Conf)
        && can->tpdo3()->configure(vars.tpdo3Conf)
        && (vars.actualControlMode = VOCAB_CM_CONFIGURED, true)
        && can->nmt()->issueServiceCommand(NmtService::START_REMOTE_NODE)
        && (can->driveStatus()->getCurrentState() != DriveState::NOT_READY_TO_SWITCH_ON
            || can->driveStatus()->awaitState(DriveState::SWITCH_ON_DISABLED))
        && can->driveStatus()->requestState(DriveState::SWITCHED_ON);
}

// -----------------------------------------------------------------------------

bool TechnosoftIpos::finalize()
{
    bool ok = true;

    if (!can->driveStatus()->requestState(DriveState::SWITCH_ON_DISABLED))
    {
        CD_WARNING("SWITCH_ON_DISABLED transition failed.\n");
        ok = false;
    }
    else
    {
        vars.actualControlMode = VOCAB_CM_CONFIGURED;
    }

    if (!can->nmt()->issueServiceCommand(NmtService::RESET_NODE))
    {
        CD_WARNING("Reset node NMT service failed.\n");
        ok = false;
    }
    else
    {
        vars.actualControlMode = VOCAB_CM_NOT_CONFIGURED;
    }

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
