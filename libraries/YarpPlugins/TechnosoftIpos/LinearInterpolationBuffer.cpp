// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "LinearInterpolationBuffer.hpp"

#include <yarp/os/Value.h>

#include <ColorDebug.h>

#include "CanUtils.hpp"

using namespace roboticslab;

LinearInterpolationBuffer::LinearInterpolationBuffer(const StateVariables & _vars, int _periodMs, int _bufferSize)
    : vars(_vars),
      periodMs(_periodMs),
      bufferSize(_bufferSize),
      integrityCounter(0)
{ }

LinearInterpolationBuffer * LinearInterpolationBuffer::createBuffer(const yarp::os::Searchable & config,
        const StateVariables & vars, unsigned int canId)
{
    double periodMs = vars.syncPeriod * 1000.0;
    int bufferSize = config.check("bufferSize", yarp::os::Value(0), "linear interpolation mode buffer size").asInt32();
    std::string mode = config.check("mode", yarp::os::Value(""), "linear interpolation mode (pt/pvt)").asString();

    if (periodMs != static_cast<int>(periodMs))
    {
        CD_ERROR("Discarded decimal values, at most millisecond precision allowed: %f (ms).\n", periodMs);
        return nullptr;
    }

    if (bufferSize <= 0)
    {
        CD_ERROR("Invalid linear interpolation mode buffer size: %d.\n", bufferSize);
        return nullptr;
    }

    if (mode == "pt")
    {
        if (bufferSize > PT_BUFFER_MAX_SIZE - 1) // consume one additional slot to avoid annoying buffer full warnings
        {
            CD_ERROR("Invalid PT mode buffer size: %d > %d.\n", bufferSize, PT_BUFFER_MAX_SIZE);
            return nullptr;
        }

        return new PtBuffer(vars, periodMs, bufferSize, canId);
    }
    else if (mode == "pvt")
    {
        if (bufferSize < 2)
        {
            CD_ERROR("Invalid PVT mode buffer size: %d < 2.\n", bufferSize);
            return nullptr;
        }
        else if (bufferSize > PVT_BUFFER_MAX_SIZE)
        {
            CD_ERROR("Invalid PVT mode buffer size: %d > %d.\n", bufferSize, PVT_BUFFER_MAX_SIZE);
            return nullptr;
        }

        return new PvtBuffer(vars, periodMs, bufferSize, canId);
    }
    else
    {
        CD_ERROR("Unsupported linear interpolation mode: \"%s\".\n", mode.c_str());
        return nullptr;
    }
}

void LinearInterpolationBuffer::resetIntegrityCounter()
{
    integrityCounter = 0;
}

int LinearInterpolationBuffer::getPeriodMs() const
{
    return periodMs;
}

int LinearInterpolationBuffer::getBufferSize() const
{
    return bufferSize;
}

PtBuffer::PtBuffer(const StateVariables & vars, int periodMs, int bufferSize, unsigned int canId)
    : LinearInterpolationBuffer(vars, periodMs, bufferSize)
{
    CD_SUCCESS("Created PT buffer with period %d (ms) and buffer size %d (canId: %d).\n", periodMs, bufferSize, canId);
}

std::string PtBuffer::getType() const
{
    return "pt";
}

std::int16_t PtBuffer::getSubMode() const
{
    return 0;
}

std::uint64_t PtBuffer::makeDataRecord(double target)
{
    std::uint64_t data = 0;

    std::int32_t position = vars.degreesToInternalUnits(target);
    data += position;

    std::int16_t time = periodMs;
    data += (std::uint64_t)time << 32;

    std::uint8_t ic = integrityCounter++ << 1;
    data += (std::uint64_t)ic << 56;

    return data;
}

PvtBuffer::PvtBuffer(const StateVariables & vars, int periodMs, int bufferSize, unsigned int canId)
    : LinearInterpolationBuffer(vars, periodMs, bufferSize),
      previousTarget(0.0),
      isFirstPoint(true)
{
    CD_SUCCESS("Created PVT buffer with period %d (ms) and buffer size %d (canId: %d).\n", periodMs, bufferSize, canId);
}

std::string PvtBuffer::getType() const
{
    return "pvt";
}

std::int16_t PvtBuffer::getSubMode() const
{
    return -1;
}

std::uint64_t PvtBuffer::makeDataRecord(double target)
{
    double v = !isFirstPoint ? (target - previousTarget) / (2 * periodMs * 0.001) : 0.0;
    std::uint64_t data = 0;

    std::int32_t position = vars.degreesToInternalUnits(target);
    std::int16_t positionLSB = (std::int32_t)(position << 16) >> 16;
    std::int8_t positionMSB = (std::int32_t)(position << 8) >> 24;
    data += ((std::uint64_t)positionMSB << 24) + positionLSB;

    double velocity = vars.degreesToInternalUnits(v, 1);
    std::int16_t velocityInt;
    std::uint16_t velocityFrac;
    CanUtils::encodeFixedPoint(velocity, &velocityInt, &velocityFrac);
    data += ((std::uint64_t)velocityInt << 32) + ((std::uint64_t)velocityFrac << 16);

    std::int16_t time = (std::int16_t)(periodMs << 7) >> 7;
    std::uint8_t ic = (integrityCounter++) << 1;
    std::uint16_t timeAndIc = time + (ic << 8);
    data += (std::uint64_t)timeAndIc << 48;

    previousTarget = target;
    isFirstPoint = false;
    return data;
}
