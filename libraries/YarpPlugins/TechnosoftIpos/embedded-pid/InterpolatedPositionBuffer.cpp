// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "InterpolatedPositionBuffer.hpp"

#include <cmath>
#include <cstring>

#include <algorithm>
#include <bitset>
#include <iterator>

#include <yarp/os/LogStream.h>
#include <yarp/os/SystemClock.h>
#include <yarp/os/Value.h>

#include "CanUtils.hpp"
#include "LogComponent.hpp"

using namespace roboticslab;

// https://github.com/roboticslab-uc3m/yarp-devices/issues/198#issuecomment-487279910
constexpr std::size_t PT_BUFFER_MAX = 9; // 285 if properly configured
constexpr std::size_t PVT_BUFFER_MAX = 7; // 222 if properly configured
constexpr std::size_t BUFFER_LOW = 4; // max: 15

InterpolatedPositionBuffer::InterpolatedPositionBuffer(double samplingPeriod, double interpolationPeriod)
    : samplingPeriod(samplingPeriod),
      fixedSamples(interpolationPeriod / samplingPeriod),
      integrityCounter(0),
      prevTarget({0, 0.0}),
      initialTimestamp(0.0),
      sampleCount(0)
{ }

void InterpolatedPositionBuffer::setInitial(int initialTarget)
{
    std::lock_guard<std::mutex> lock(queueMutex);
    initialTimestamp = 0.0; // dummy timestamp, to be amended later on
    prevTarget = {initialTarget, initialTimestamp};
    sampleCount = 0;
}

int InterpolatedPositionBuffer::getPeriodMs() const
{
    return fixedSamples * samplingPeriod * 1000.0;
}

std::uint16_t InterpolatedPositionBuffer::getBufferConfig() const
{
    std::bitset<16> bits("1011000010000000"); // 0xB080
    bits |= (BUFFER_LOW << 8);
    bits |= ((integrityCounter << 1) >> 1);
    return bits.to_ulong();
}

void InterpolatedPositionBuffer::addSetpoint(int target)
{
    std::lock_guard<std::mutex> lock(queueMutex);
    pendingTargets.emplace_back(target, yarp::os::SystemClock::nowSystem());
}

std::vector<std::uint64_t> InterpolatedPositionBuffer::popBatch(bool fullBuffer)
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
            ip_record nextTarget = !pendingTargets.empty() ? pendingTargets.front() : std::make_pair(0, 0.0);

            auto setpoint = makeDataRecord(prevTarget, currTarget, nextTarget);

            prevTarget = currTarget;
            integrityCounter++;
            return setpoint;
        });

    return batch;
}

int InterpolatedPositionBuffer::getPrevTarget() const
{
    std::lock_guard<std::mutex> lock(queueMutex);
    return prevTarget.first;
}

bool InterpolatedPositionBuffer::isQueueReady() const
{
    std::lock_guard<std::mutex> lock(queueMutex);
    // account for the offset (only PVT) and one extra point in async mode (`>` instead of `>=`)
    return pendingTargets.size() > getBufferSize() + getOffset();
}

bool InterpolatedPositionBuffer::isQueueEmpty() const
{
    std::lock_guard<std::mutex> lock(queueMutex);
    return pendingTargets.empty();
}

void InterpolatedPositionBuffer::clearQueue()
{
    std::lock_guard<std::mutex> lock(queueMutex);
    pendingTargets.clear();
}

std::uint8_t InterpolatedPositionBuffer::getIntegrityCounter() const
{
    return integrityCounter;
}

std::size_t InterpolatedPositionBuffer::getOffset() const
{
    return 0;
}

std::uint16_t InterpolatedPositionBuffer::getSampledTime(double currentTimestamp)
{
    if (fixedSamples != 0)
    {
        // Synchronous interpolation.
        return fixedSamples;
    }

    // Asynchronous interpolation.
    double elapsed = currentTimestamp - initialTimestamp;
    int samplesSinceStart = elapsed / samplingPeriod;
    std::uint16_t currentWindow = samplesSinceStart - sampleCount;
    sampleCount = samplesSinceStart;
    return currentWindow;
}

double InterpolatedPositionBuffer::getMeanVelocity(const ip_record & earliest, const ip_record & latest) const
{
    double distance = latest.first - earliest.first;

    if (fixedSamples != 0)
    {
        // Synchronous interpolation.
        return distance / (fixedSamples * samplingPeriod);
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

    std::int32_t p = current.first;
    std::uint16_t t = getSampledTime(current.second);
    std::uint8_t ic = getIntegrityCounter() << 1;

    std::memcpy((unsigned char *)&data, &p, sizeof(p));
    std::memcpy((unsigned char *)&data + 4, &t, sizeof(t));
    std::memcpy((unsigned char *)&data + 7, &ic, sizeof(ic));

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

    std::int32_t position = current.first;
    std::int16_t p_lsb = position & 0x0000FFFF;
    std::int8_t p_msb = ((position & 0x00FF0000) << 8) >> 24;

    std::memcpy((unsigned char *)&data, &p_lsb, sizeof(p_lsb));
    std::memcpy((unsigned char *)&data + 3, &p_msb, sizeof(p_msb));

    if (next.second != 0.0)
    {
        double prevVelocity = getMeanVelocity(previous, current);
        double nextVelocity = getMeanVelocity(current, next);
        double velocity = (prevVelocity + nextVelocity) * samplingPeriod / 2.0;

        std::int16_t v_int;
        std::uint8_t v_frac;

        CanUtils::encodeFixedPoint(velocity, &v_int, &v_frac);

        std::memcpy((unsigned char *)&data + 2, &v_frac, sizeof(v_frac));
        std::memcpy((unsigned char *)&data + 4, &v_int, sizeof(v_int));
    }

    std::uint16_t t = getSampledTime(current.second) & 0x1FF;
    std::uint8_t ic = getIntegrityCounter() << 1;
    std::uint16_t tic = t + (static_cast<std::uint16_t>(ic) << 8);

    std::memcpy((unsigned char *)&data + 6, &tic, sizeof(tic));

    return data;
}

namespace roboticslab
{

InterpolatedPositionBuffer * createInterpolationBuffer(const yarp::os::Searchable & config, double samplingPeriod)
{
    std::string mode = config.check("mode", yarp::os::Value(""), "interpolated position submode [pt|pvt]").asString();
    int periodMs = config.check("periodMs", yarp::os::Value(0), "interpolated position fixed period (ms)").asInt32();

    if (periodMs < 0)
    {
        yCError(IPOS) << "Illegal \"periodMs\":" << periodMs;
        return nullptr;
    }

    if (mode == "pt")
    {
        return new PtBuffer(samplingPeriod, periodMs * 0.001);
    }
    else if (mode == "pvt")
    {
        return new PvtBuffer(samplingPeriod, periodMs * 0.001);
    }
    else
    {
        yCError(IPOS) << "Unsupported interpolated position submode:" << mode;
        return nullptr;
    }
}

} // namespace roboticslab
