// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "LinearInterpolationBuffer.hpp"

#include <cmath>

#include <algorithm>
#include <bitset>
#include <iterator>

#include <yarp/os/Value.h>

#include <ColorDebug.h>

#include "CanUtils.hpp"

using namespace roboticslab;

// https://github.com/roboticslab-uc3m/yarp-devices/issues/198#issuecomment-487279910
const unsigned int LinearInterpolationBuffer::PT_BUFFER_MAX = 9; // 285 if properly configured
const unsigned int LinearInterpolationBuffer::PVT_BUFFER_MAX = 7; // 222 if properly configured
const unsigned int LinearInterpolationBuffer::BUFFER_LOW = 4;// max: 15

LinearInterpolationBuffer::LinearInterpolationBuffer(const StateVariables & _vars, std::uint16_t _periodMs)
    : vars(_vars),
      periodMs(_periodMs),
      integrityCounter(0),
      motionStarted(false)
{ }

LinearInterpolationBuffer * LinearInterpolationBuffer::createBuffer(const yarp::os::Searchable & config,
        const StateVariables & vars)
{
    unsigned int periodMs = config.check("periodMs", yarp::os::Value(0), "pt/pvt time (ms)").asInt32();
    std::string mode = config.check("mode", yarp::os::Value(""), "linear interpolation mode [pt|pvt]").asString();

    if (periodMs <= 0)
    {
        CD_ERROR("Illegal \"periodMs\": %d.\n", periodMs);
        return nullptr;
    }

    if (mode == "pt")
    {
        return new PtBuffer(vars, periodMs);
    }
    else if (mode == "pvt")
    {
        return new PvtBuffer(vars, periodMs);
    }
    else
    {
        CD_ERROR("Unsupported linear interpolation mode: \"%s\".\n", mode.c_str());
        return nullptr;
    }
}

void LinearInterpolationBuffer::reset()
{
    std::lock_guard<std::mutex> lock(queueMutex);
    integrityCounter = 0;
    pendingSetpoints.clear();
}

std::uint16_t LinearInterpolationBuffer::getPeriodMs() const
{
    return periodMs;
}

unsigned int LinearInterpolationBuffer::getQueueSize() const
{
    std::lock_guard<std::mutex> lock(queueMutex);
    return pendingSetpoints.size();
}

std::uint16_t LinearInterpolationBuffer::getBufferConfig() const
{
    std::bitset<16> bits("1010000010001000"); // 0xA088
    bits |= (BUFFER_LOW << 8);
    bits |= ((integrityCounter << 1) >> 1);
    return bits.to_ulong();
}

void LinearInterpolationBuffer::addSetpoint(double target)
{
    std::lock_guard<std::mutex> lock(queueMutex);
    pendingSetpoints.push_back(makeDataRecord(target));
    integrityCounter++;
}

std::vector<std::uint64_t> LinearInterpolationBuffer::popBatch(bool fullBuffer)
{
    const int count = fullBuffer ? getBufferSize() : getBufferSize() - BUFFER_LOW;
    std::vector<std::uint64_t> batch;
    std::lock_guard<std::mutex> lock(queueMutex);

    std::generate_n(std::back_inserter(batch), std::min<int>(count, pendingSetpoints.size()),
            [this]
            {
                auto data = pendingSetpoints.front();
                pendingSetpoints.pop_front();
                return data;
            });

    return batch;
}

bool LinearInterpolationBuffer::isStarted() const
{
    return motionStarted;
}

void LinearInterpolationBuffer::reportMotionStatus(bool isStarted)
{
    motionStarted = isStarted;
}

std::uint8_t LinearInterpolationBuffer::getIntegrityCounter() const
{
    return integrityCounter;
}

std::string PtBuffer::getType() const
{
    return "pt";
}

std::uint16_t PtBuffer::getBufferSize() const
{
    return PT_BUFFER_MAX;
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

    std::uint8_t ic = getIntegrityCounter() << 1;
    data += (std::uint64_t)ic << 56;

    return data;
}

std::string PvtBuffer::getType() const
{
    return "pvt";
}

std::uint16_t PvtBuffer::getBufferSize() const
{
    return PVT_BUFFER_MAX;
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
    std::uint8_t ic = getIntegrityCounter() << 1;
    std::uint16_t timeAndIc = time + (ic << 8);
    data += (std::uint64_t)timeAndIc << 48;

    previousTarget = target;
    isFirstPoint = false;
    return data;
}
