// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TechnosoftIposBase.hpp"

#include <functional>

#include <yarp/os/LogStream.h>
#include <yarp/os/Property.h>

#include "LogComponent.hpp"
#include "TechnosoftIposEmcy.hpp"

using namespace roboticslab;


// -----------------------------------------------------------------------------

bool TechnosoftIposBase::open(yarp::os::Searchable & config)
{
    actualControlMode = VOCAB_CM_NOT_CONFIGURED;

    if (!validateInitialState(params, id()))
    {
        yCIError(IPOS, id()) << "Invalid configuration parameters";
        return false;
    }

    jointType = yarp::os::Vocab32::encode(params.m_type);
    max = params.m_max;
    min = params.m_min;
    maxVel = params.m_maxVel;
    refSpeed = params.m_refSpeed;
    refAcceleration = params.m_refAcceleration;
    k = params.m_motor_k;
    tr = params.m_gearbox_tr * params.m_extraTr;
    encoderPulses = params.m_encoder_encoderPulses;
    initialControlMode = yarp::os::Vocab32::encode(params.m_initialControlMode);

    if (config.check("externalEncoder", "external encoder"))
    {
        // TODO: this will go away in attach mode and no longer depend on upstream `config`

        const auto * robotConfig = *reinterpret_cast<const yarp::os::Property * const *>(config.find("robotConfig").asBlob());

        std::string externalEncoder = config.find("externalEncoder").asString();
        yarp::os::Bottle & externalEncoderGroup = robotConfig->findGroup(externalEncoder);

        if (externalEncoderGroup.isNull())
        {
            yCIError(IPOS, id()) << "Missing external encoder device group" << externalEncoder;
            return false;
        }

        yarp::os::Property externalEncoderOptions;
        externalEncoderOptions.fromString(externalEncoderGroup.toString());
        externalEncoderOptions.put("robotConfig", config.find("robotConfig"));

        if (!externalEncoderDevice.open(externalEncoderOptions))
        {
            yCIError(IPOS, id()) << "Unable to open external encoder device" << externalEncoder;
            return false;
        }

        if (!externalEncoderDevice.view(iEncodersTimedRawExternal))
        {
            yCIError(IPOS, id()) << "Unable to view IEncodersTimedRaw in" << externalEncoder;
            return false;
        }

        if (!externalEncoderDevice.view(iExternalEncoderCanBusSharer))
        {
            yCIError(IPOS, id()) << "Unable to view ICanBusSharer in" << externalEncoder;
            return false;
        }
    }

    can = new CanOpenNode(params.m_canId, params.m_sdoTimeout, params.m_driveStateTimeout);

    // Manufacturer Status Register (1002h) and Modes of Operation Display (6061h)
    tpdo1Conf.addMapping<std::uint32_t>(0x1002).addMapping<std::int8_t>(0x6061);

    if (params.m_tpdo1InhibitTime > 0.0)
    {
        tpdo1Conf.setInhibitTime(params.m_tpdo1InhibitTime * 1e4); // pass x100 microseconds
    }

    if (params.m_tpdo1EventTimer > 0.0)
    {
        tpdo1Conf.setEventTimer(params.m_tpdo1EventTimer * 1e3); // pass milliseconds
    }

    // Motion Error Register (2000h) and Detailed Error Register (2002h)
    tpdo2Conf.addMapping<std::uint16_t>(0x2000).addMapping<std::uint16_t>(0x2002);

    if (params.m_tpdo2InhibitTime > 0.0)
    {
        tpdo2Conf.setInhibitTime(params.m_tpdo2InhibitTime * 1e4); // pass x100 microseconds
    }

    if (params.m_tpdo2EventTimer > 0.0)
    {
        tpdo2Conf.setEventTimer(params.m_tpdo2EventTimer * 1e3); // pass milliseconds
    }

    // Position actual internal value (6063h) and Torque actual value (6077h)
    tpdo3Conf.addMapping<std::int32_t>(0x6063).addMapping<std::int16_t>(0x6077);

    tpdo3Conf.setTransmissionType(PdoTransmissionType::SYNCHRONOUS_CYCLIC);

    using namespace std::placeholders;

    can->tpdo1()->registerHandler<std::uint16_t, std::uint16_t, std::int8_t>(std::bind(&TechnosoftIposBase::handleTpdo1, this, _1, _2, _3));
    can->tpdo2()->registerHandler<std::uint16_t, std::uint16_t>(std::bind(&TechnosoftIposBase::handleTpdo2, this, _1, _2));
    can->tpdo3()->registerHandler<std::int32_t, std::int16_t>(std::bind(&TechnosoftIposBase::handleTpdo3, this, _1, _2));

    can->emcy()->registerHandler(std::bind(&TechnosoftIposBase::handleEmcy, this, _1, _2, _3));
    can->emcy()->setErrorCodeRegistry<TechnosoftIposEmcy>();

    can->nmt()->registerHandler(std::bind(&TechnosoftIposBase::handleNmt, this, _1));

    if (params.m_monitorPeriod > 0.0)
    {
        monitorThread = new yarp::os::Timer(yarp::os::TimerSettings(params.m_monitorPeriod), std::bind(&TechnosoftIposBase::monitorWorker, this, _1), false);
    }

    lastEncoderRead = std::make_unique<EncoderRead>(params.m_samplingPeriod);

    return !monitorThread || monitorThread->start();
}

// -----------------------------------------------------------------------------

bool TechnosoftIposBase::close()
{
    // we need to do this in finalize(), too, since the monitor thread could be
    // still requesting CAN transfers even after CAN RX/TX threads have been
    // closed in CanBusBroker::close()
    if (monitorThread && monitorThread->isRunning())
    {
        monitorThread->stop();
    }

    delete monitorThread;
    monitorThread = nullptr;

    delete can;
    can = nullptr;

    if (externalEncoderDevice.isValid())
    {
        return externalEncoderDevice.close();
    }

    return true;
}

// -----------------------------------------------------------------------------
