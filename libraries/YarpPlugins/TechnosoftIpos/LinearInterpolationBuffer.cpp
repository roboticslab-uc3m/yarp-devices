// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "LinearInterpolationBuffer.hpp"

#include <cmath>

#include <algorithm>
#include <bitset>
#include <iterator>

#include <yarp/os/Time.h>
#include <yarp/os/Value.h>

#include <ColorDebug.h>

#include "CanUtils.hpp"

using namespace roboticslab;

namespace
{
    // https://github.com/roboticslab-uc3m/yarp-devices/issues/198#issuecomment-487279910
    constexpr std::size_t PT_BUFFER_MAX = 9; // 285 if properly configured
    constexpr std::size_t PVT_BUFFER_MAX = 7; // 222 if properly configured
    constexpr std::size_t BUFFER_LOW = 4; // max: 15
}

LinearInterpolationBuffer::LinearInterpolationBuffer(const StateVariables & _vars, int periodMs)
    : vars(_vars),
      fixedSamples(periodMs * 0.001 / vars.samplingPeriod),
      integrityCounter(0),
      prevTarget({0.0, 0.0}),
      initialTimestamp(0.0),
      sampleCount(0)
{ }

void LinearInterpolationBuffer::setInitial(double initialTarget)
{
    std::lock_guard<std::mutex> lock(queueMutex);
    initialTimestamp = 0.0; // dummy timestamp, to be amended later on
    prevTarget = {initialTarget, initialTimestamp};
    sampleCount = 0;
}

int LinearInterpolationBuffer::getPeriodMs() const
{
    return fixedSamples * vars.samplingPeriod * 1000.0;
}

std::uint16_t LinearInterpolationBuffer::getBufferConfig() const
{
    std::bitset<16> bits("1011000010000000"); // 0xB080
    bits |= (BUFFER_LOW << 8);
    bits |= ((integrityCounter << 1) >> 1);
    return bits.to_ulong();
}

void LinearInterpolationBuffer::addSetpoint(double target)
{
    std::lock_guard<std::mutex> lock(queueMutex);
    pendingTargets.push_back({target, yarp::os::Time::now()});
}

std::vector<std::uint64_t> LinearInterpolationBuffer::popBatch(bool fullBuffer)
{
    std::lock_guard<std::mutex> lock(queueMutex);

    if (pendingTargets.empty())
    {
        return {};
    }

    if (prevTarget.second == 0.0 && pendingTargets.size() > 1)
    {
        // Prior to initializing motion, prevTarget is a dummy setpoint referring to current
        // joint position. Infer timestamp from first two stored points.
        auto diff = pendingTargets[1].second - pendingTargets[0].second;
        initialTimestamp = prevTarget.second = pendingTargets[0].second - diff;
    }

    // This method may be called in any of these two circumstances:
    // - fullBuffer == true: fill the whole HW buffer on motion start, send as many points as possible.
    // - fullBuffer == false: replenish HW buffer on buffer-low signal, substract fixed threshold from max size.
    const std::size_t batchSize = fullBuffer ? getBufferSize() : getBufferSize() - BUFFER_LOW;

    std::vector<std::uint64_t> batch;
    batch.reserve(batchSize);

    // This correction is necessary to make sure that pendingTargets is fully processed in case
    // there is exactly one point left, even if the submode (i.e. PVT) uses an intrinsic offset.
    const std::size_t pending = std::max(pendingTargets.size() - getOffset(), getOffset());

    std::generate_n(std::back_inserter(batch), std::min(batchSize, pending),
        [this]
        {
            // Pop first point from queue (FIFO).
            ip_record currTarget = pendingTargets.front();
            pendingTargets.pop_front();

            // If available, capture next point, otherwise default to empty pair.
            ip_record nextTarget = !pendingTargets.empty() ? pendingTargets.front() : std::make_pair(0.0, 0.0);

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
    return prevTarget.first;
}

bool LinearInterpolationBuffer::isQueueReady() const
{
    std::lock_guard<std::mutex> lock(queueMutex);
    return pendingTargets.size() >= getBufferSize() + getOffset();
}

bool LinearInterpolationBuffer::isQueueEmpty() const
{
    std::lock_guard<std::mutex> lock(queueMutex);
    return pendingTargets.empty();
}

std::uint8_t LinearInterpolationBuffer::getIntegrityCounter() const
{
    return integrityCounter;
}

std::size_t LinearInterpolationBuffer::getOffset() const
{
    return 0;
}

std::uint16_t LinearInterpolationBuffer::getSampledTime(double currentTimestamp)
{
    if (fixedSamples != 0)
    {
        // Synchronous interpolation.
        return fixedSamples;
    }

    // Asynchronous interpolation.
    double elapsed = currentTimestamp - initialTimestamp;
    int samplesSinceStart = elapsed / vars.samplingPeriod;
    std::uint16_t currentWindow = samplesSinceStart - sampleCount;
    sampleCount = samplesSinceStart;
    return currentWindow;
}

double LinearInterpolationBuffer::getMeanVelocity(const ip_record & earliest, const ip_record & latest) const
{
    double distance = latest.first - earliest.first;

    if (fixedSamples != 0)
    {
        // Synchronous interpolation.
        return distance / (fixedSamples * vars.samplingPeriod);
    }
    else
    {
        // Asynchronous interpolation.
        return distance / (latest.second - earliest.second);
    }
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

std::uint64_t PtBuffer::makeDataRecord(const ip_record & previous, const ip_record & current, const ip_record & next)
{
    std::uint64_t data = 0;
    data += static_cast<std::int32_t>(vars.degreesToInternalUnits(current.first));
    data += static_cast<std::uint64_t>(getSampledTime(current.second)) << 32;
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

std::size_t PvtBuffer::getOffset() const
{
    return 1;
}

std::uint64_t PvtBuffer::makeDataRecord(const ip_record & previous, const ip_record & current, const ip_record & next)
{
    std::uint64_t data = 0;

    std::int32_t position = vars.degreesToInternalUnits(current.first);
    data += ((position & 0x00FF0000) << 8) + (position & 0x0000FFFF);

    if (next.second != 0.0)
    {
        double prevVelocity = getMeanVelocity(previous, current);
        double nextVelocity = getMeanVelocity(current, next);
        double velocity = vars.degreesToInternalUnits((prevVelocity + nextVelocity) / 2.0, 1);

        std::int16_t velocityInt;
        std::uint8_t velocityFrac;
        CanUtils::encodeFixedPoint(velocity, &velocityInt, &velocityFrac);
        data += static_cast<std::uint64_t>(velocityFrac) << 16;
        data += static_cast<std::uint64_t>(velocityInt) << 32;
    }

    data += static_cast<std::uint64_t>(getSampledTime(current.second) & 0x1FF) << 48;
    data += static_cast<std::uint64_t>(getIntegrityCounter()) << 57;

    return data;
}

namespace roboticslab
{

LinearInterpolationBuffer * createInterpolationBuffer(const yarp::os::Searchable & config, const StateVariables & vars)
{
    std::string mode = config.check("mode", yarp::os::Value(""), "linear interpolation mode [pt|pvt]").asString();
    int periodMs = config.check("periodMs", yarp::os::Value(0), "linear interpolation fixed period (ms)").asInt32();

    if (periodMs < 0)
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
