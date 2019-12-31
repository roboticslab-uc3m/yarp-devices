// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "LinearInterpolationBuffer.hpp"

#include <cmath>

#include "TechnosoftIpos.hpp"
#include "CanUtils.hpp"

#include <ColorDebug.h>

using namespace roboticslab;

LinearInterpolationBuffer::LinearInterpolationBuffer(int _periodMs, int _bufferSize, double _factor, double _maxVel)
    : periodMs(_periodMs),
      bufferSize(_bufferSize),
      factor(_factor),
      maxVel(_maxVel),
      lastSentTarget(0.0),
      lastReceivedTarget(0.0),
      integrityCounter(0)
{}

LinearInterpolationBuffer * LinearInterpolationBuffer::createBuffer(const yarp::os::Searchable & config,
        const StateVariables & vars)
{
    int linInterpPeriodMs = config.check("linInterpPeriodMs", yarp::os::Value(DEFAULT_PERIOD_MS),
            "linear interpolation mode period (ms)").asInt32();
    int linInterpBufferSize = config.check("linInterpBufferSize", yarp::os::Value(DEFAULT_BUFFER_SIZE),
            "linear interpolation mode buffer size").asInt32();
    std::string linInterpMode = config.check("linInterpMode", yarp::os::Value(DEFAULT_MODE),
            "linear interpolation mode (pt/pvt)").asString();

    if (linInterpPeriodMs <= 0)
    {
        CD_ERROR("Invalid linear interpolation mode period: %d.\n", linInterpPeriodMs);
        return nullptr;
    }

    if (linInterpBufferSize <= 0)
    {
        CD_ERROR("Invalid linear interpolation mode buffer size: %d.\n", linInterpBufferSize);
        return nullptr;
    }

    LinearInterpolationBuffer * buff;

    if (linInterpMode == "pt")
    {
        if (linInterpBufferSize > PT_BUFFER_MAX_SIZE - 1) // consume one additional slot to avoid annoying buffer full warnings
        {
            CD_ERROR("Invalid PT mode buffer size: %d > %d.\n", linInterpBufferSize, PT_BUFFER_MAX_SIZE);
            return nullptr;
        }

        return new PtBuffer(linInterpPeriodMs, linInterpBufferSize, vars.degreesToInternalUnits(1.0), vars.maxVel);
    }
    else if (linInterpMode == "pvt")
    {
        if (linInterpBufferSize < 2)
        {
            CD_ERROR("Invalid PVT mode buffer size: %d < 2.\n", linInterpBufferSize);
            return nullptr;
        }
        else if (linInterpBufferSize > PVT_BUFFER_MAX_SIZE)
        {
            CD_ERROR("Invalid PVT mode buffer size: %d > %d.\n", linInterpBufferSize, PVT_BUFFER_MAX_SIZE);
            return nullptr;
        }

        return new PvtBuffer(linInterpPeriodMs, linInterpBufferSize, vars.degreesToInternalUnits(1.0), vars.maxVel);
    }
    else
    {
        CD_ERROR("Unsupported linear interpolation mode: \"%s\".\n", linInterpMode.c_str());
        return nullptr;
    }
}

void LinearInterpolationBuffer::resetIntegrityCounter()
{
    integrityCounter = 0;
}

void LinearInterpolationBuffer::setInitialReference(double target)
{
    lastSentTarget = target;
}

void LinearInterpolationBuffer::updateTarget(double target)
{
    std::lock_guard<std::mutex> lock(mutex);
    lastReceivedTarget = target;
}

double LinearInterpolationBuffer::getLastTarget() const
{
    std::lock_guard<std::mutex> lock(mutex);
    return lastSentTarget;
}

int LinearInterpolationBuffer::getPeriod() const
{
    return periodMs;
}

void LinearInterpolationBuffer::setPeriod(int periodMs)
{
    this->periodMs = periodMs;
}

int LinearInterpolationBuffer::getBufferSize() const
{
    return bufferSize;
}

void LinearInterpolationBuffer::setBufferSize(int bufferSize)
{
    this->bufferSize = bufferSize;
}

LinearInterpolationBuffer * LinearInterpolationBuffer::cloneTo(const std::string & type)
{
    LinearInterpolationBuffer * buffer;

    if (type == "pt")
    {
        buffer = new PtBuffer(periodMs, bufferSize, factor, maxVel);
    }
    else if (type == "pvt")
    {
        buffer = new PvtBuffer(periodMs, bufferSize, factor, maxVel);
    }
    else
    {
        CD_ERROR("Unsupported linear interpolation mode: \"%s\".\n", type.c_str());
        return nullptr;
    }

    return buffer;
}

PtBuffer::PtBuffer(int _periodMs, int _bufferSize, double _factor, double _maxVel)
    : LinearInterpolationBuffer(_periodMs, _bufferSize, _factor, _maxVel),
      maxDistance(_maxVel * _periodMs * 0.001)
{
    CD_SUCCESS("Created PT buffer with period %d (ms) and buffer size %d.\n", periodMs, bufferSize);
}

std::string PtBuffer::getType() const
{
    return "pt";
}

std::int16_t PtBuffer::getSubMode() const
{
    return 0;
}

std::uint64_t PtBuffer::makeDataRecord()
{
    //*************************************************************
    //-- 14. Send the 1 st PT point.
    //-- Position= 20000 IU (0x00004E20) 1IU = 1 encoder pulse
    //-- Time = 1000 IU (0x03E8)
    //-- IC = 1 (0x01)
    //-- 1IU = 1 control loop = 1ms by default
    //-- IC=Integrity Counter
    //-- The drive motor will do 10 rotations (20000 counts) in 1000 milliseconds.
    //-- Send the following message:
    //uint8_t ptpoint1[]={0x20,0x4E,0x00,0x00,0xE8,0x03,0x00,0x02};

    mutex.lock();
    double _lastReceivedTarget = lastReceivedTarget;
    mutex.unlock();

    if (std::abs(_lastReceivedTarget - lastSentTarget) > maxDistance)
    {
        CD_WARNING("Max velocity exceeded, clipping travelled distance.\n");
        _lastReceivedTarget = lastSentTarget + maxDistance * (_lastReceivedTarget >= lastSentTarget ? 1 : -1);
        CD_INFO("New ref: %f.\n", _lastReceivedTarget);
    }

    std::uint64_t data = 0;

    double p = _lastReceivedTarget;
    int t = periodMs;

    std::int32_t position = p * factor;
    data += position;

    std::int16_t time = t;
    data += (std::uint64_t)time << 32;

    std::uint8_t ic = integrityCounter++ << 1;
    data += (std::uint64_t)ic << 56;

    CD_DEBUG("Sending p %f t %d (ic %d).\n", p, t, ic >> 1);

    lastSentTarget = _lastReceivedTarget;

    return data;
}

PvtBuffer::PvtBuffer(int _periodMs, int _bufferSize, double _factor, double _maxVel)
    : LinearInterpolationBuffer(_periodMs, _bufferSize, _factor, _maxVel),
      previousTarget(0.0),
      isFirstPoint(true)
{
    CD_SUCCESS("Created PVT buffer with period %d (ms) and buffer size %d.\n", periodMs, bufferSize);
}

void PvtBuffer::setInitialReference(double target)
{
    LinearInterpolationBuffer::setInitialReference(target);
    previousTarget = target;
}

std::string PvtBuffer::getType() const
{
    return "pvt";
}

std::int16_t PvtBuffer::getSubMode() const
{
    return -1;
}

std::uint64_t PvtBuffer::makeDataRecord()
{
    //*************************************************************
    //-- 13. Send the 1 st PT point.
    //-- Position = 88 IU (0x000058) 1IU = 1 encoder pulse
    //-- Velocity = 3.33 IU (0x000354) 1IU = 1 encoder pulse/ 1 control loop
    //-- Time = 55 IU (0x37) 1IU = 1 control loop = 1ms by default
    //-- IC = 0 (0x00) IC=Integrity Counter
    //-- Send the following message:
    //uint8_t ptpoint1[]={0x58,0x00,0x54,0x00,0x03,0x00,0x37,0x00};

    mutex.lock();
    double currentTarget = lastReceivedTarget;
    mutex.unlock();

    double p = previousTarget;
    double v;
    int t = periodMs;

    if (!isFirstPoint)
    {
        v = (currentTarget - lastSentTarget) / (2 * periodMs * 0.001);
    }
    else
    {
        v = 0.0;
        isFirstPoint = false;
    }

    std::uint64_t data = 0;

    std::int32_t position = p * factor;
    std::int16_t positionLSB = (std::int32_t)(position << 16) >> 16;
    std::int8_t positionMSB = (std::int32_t)(position << 8) >> 24;
    data += ((std::uint64_t)positionMSB << 24) + positionLSB;

    double velocity = v * factor * 0.001;
    std::int16_t velocityInt;
    std::uint16_t velocityFrac;
    CanUtils::encodeFixedPoint(velocity, &velocityInt, &velocityFrac);
    data += ((std::uint64_t)velocityInt << 32) + ((std::uint64_t)velocityFrac << 16);

    std::int16_t time = (std::int16_t)(t << 7) >> 7;
    std::uint8_t ic = (integrityCounter++) << 1;
    std::uint16_t timeAndIc = time + (ic << 8);
    data += (std::uint64_t)timeAndIc << 48;

    CD_DEBUG("Sending p %f v %f t %d (ic %d).\n", p, v, t, ic >> 1);

    lastSentTarget = previousTarget;
    previousTarget = currentTarget;

    return data;
}
