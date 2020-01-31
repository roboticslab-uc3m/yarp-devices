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
      prevTarget(0.0)
{ }

void LinearInterpolationBuffer::init(double initialTarget)
{
    std::lock_guard<std::mutex> lock(queueMutex);
    integrityCounter = 0;
    prevTarget = initialTarget;
    pendingTargets.clear();
}

std::uint16_t LinearInterpolationBuffer::getPeriodMs() const
{
    return periodMs;
}

std::uint16_t LinearInterpolationBuffer::getBufferConfig() const
{
    std::bitset<16> bits("1011000010000000"); // 0xA080
    bits |= (BUFFER_LOW << 8);
    bits |= ((integrityCounter << 1) >> 1);
    return bits.to_ulong();
}

void LinearInterpolationBuffer::addSetpoint(double target)
{
    std::lock_guard<std::mutex> lock(queueMutex);
    pendingTargets.push_back(target);
}

std::vector<std::uint64_t> LinearInterpolationBuffer::popBatch(bool fullBuffer)
{
    const int batchSize = fullBuffer ? getBufferSize() : getBufferSize() - BUFFER_LOW;
    int offset = getType() == "pvt" ? 1 : 0;

    std::vector<std::uint64_t> batch;
    std::lock_guard<std::mutex> lock(queueMutex);

    std::generate_n(std::back_inserter(batch), std::min<int>(batchSize, pendingTargets.size() - offset),
            [this]
            {
                auto currTarget = pendingTargets.front();
                pendingTargets.pop_front();
                double nextTarget = 0.0;

                if (!pendingTargets.empty())
                {
                    if (pendingTargets.size() == 1)
                    {
                        nextTarget = currTarget; // last point, assume velocity equal to zero
                    }
                    else
                    {
                        nextTarget = pendingTargets.front();
                    }
                }

                auto setpoint = makeDataRecord(prevTarget, currTarget, nextTarget);
                prevTarget = currTarget;
                integrityCounter++;
                return setpoint;
            });

    return batch;
}

double LinearInterpolationBuffer::getPrevTarget() const
{
    std::lock_guard<std::mutex> lock(queueMutex);
    return prevTarget;
}

bool LinearInterpolationBuffer::isQueueReady() const
{
    int offset = getType() == "pvt" ? 1 : 0;
    std::lock_guard<std::mutex> lock(queueMutex);
    return pendingTargets.size() >= getBufferSize() + offset;
}

bool LinearInterpolationBuffer::isQueueEmpty() const
{
    int offset = getType() == "pvt" ? 1 : 0;
    std::lock_guard<std::mutex> lock(queueMutex);
    return pendingTargets.empty() || pendingTargets.size() <= offset;
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

std::uint64_t PtBuffer::makeDataRecord(double previous, double current, double next)
{
    std::uint64_t data = 0;
    data += static_cast<std::int32_t>(vars.degreesToInternalUnits(current));
    data += static_cast<std::uint64_t>(getPeriodMs()) << 32;
    data += static_cast<std::uint64_t>(getIntegrityCounter()) << 57;
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

std::uint64_t PvtBuffer::makeDataRecord(double previous, double current, double next)
{
    std::uint64_t data = 0;

    std::int32_t position = vars.degreesToInternalUnits(current);
    data += ((position & 0x00FF0000) << 8) + (position & 0x0000FFFF);

    if (std::abs(next - current) > 1e-6)
    {
        double prevVelocity = (current - previous) / (getPeriodMs() * 0.001);
        double nextVelocity = (next - current) / (getPeriodMs() * 0.001);
        double velocity = vars.degreesToInternalUnits((prevVelocity + nextVelocity) / 2.0, 1);

        std::int16_t velocityInt;
        std::uint8_t velocityFrac;
        CanUtils::encodeFixedPoint(velocity, &velocityInt, &velocityFrac);
        data += static_cast<std::uint64_t>(velocityFrac) << 16;
        data += static_cast<std::uint64_t>(velocityInt) << 32;
    }

    data += static_cast<std::uint64_t>(getPeriodMs() & 0x1FF) << 48;
    data += static_cast<std::uint64_t>(getIntegrityCounter()) << 57;

    return data;
}

namespace roboticslab
{

LinearInterpolationBuffer * createInterpolationBuffer(const yarp::os::Searchable & config, const StateVariables & vars)
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

} // namespace roboticslab
