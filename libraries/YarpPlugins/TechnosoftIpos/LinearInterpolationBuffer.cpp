// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "LinearInterpolationBuffer.hpp"

#include <cmath>

#include "TechnosoftIpos.hpp"

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

LinearInterpolationBuffer * LinearInterpolationBuffer::createBuffer(const yarp::os::Searchable & config)
{
    int linInterpPeriodMs = config.check("linInterpPeriodMs", yarp::os::Value(0),
            "linear interpolation mode period (ms)").asInt32();
    int linInterpBufferSize = config.check("linInterpBufferSize", yarp::os::Value(0),
            "linear interpolation mode buffer size").asInt32();
    std::string linInterpMode = config.check("linInterpMode", yarp::os::Value(""),
            "linear interpolation mode (PT/PVT)").asString();

    double tr = config.check("tr", yarp::os::Value(0.0)).asFloat64();
    int encoderPulses = config.check("encoderPulses", yarp::os::Value(0)).asInt32();
    double maxVel = config.check("maxVel", yarp::os::Value(10.0)).asFloat64();

    double factor = tr * (encoderPulses / 360.0);

    if (linInterpPeriodMs <= 0)
    {
        CD_ERROR("Invalid linear interpolation mode period: %d.\n", linInterpPeriodMs);
        return 0;
    }

    if (linInterpBufferSize <= 0)
    {
        CD_ERROR("Invalid linear interpolation mode buffer size: %d.\n", linInterpBufferSize);
        return 0;
    }

    LinearInterpolationBuffer * buff;

    if (linInterpMode == "pt")
    {
        if (linInterpBufferSize > PT_BUFFER_MAX_SIZE)
        {
            CD_ERROR("Invalid PT mode buffer size: %d > %d.\n", linInterpBufferSize, PT_BUFFER_MAX_SIZE);
            return 0;
        }

        return new PtBuffer(linInterpPeriodMs, linInterpBufferSize, factor, maxVel);
    }
    else if (linInterpMode == "pvt")
    {
        if (linInterpBufferSize < 2)
        {
            CD_ERROR("Invalid PVT mode buffer size: %d < 2.\n", linInterpBufferSize);
            return 0;
        }
        else if (linInterpBufferSize > PVT_BUFFER_MAX_SIZE)
        {
            CD_ERROR("Invalid PVT mode buffer size: %d > %d.\n", linInterpBufferSize, PVT_BUFFER_MAX_SIZE);
            return 0;
        }

        return new PvtBuffer(linInterpPeriodMs, linInterpBufferSize, factor, maxVel);
    }
    else
    {
        CD_ERROR("Unsupported linear interpolation mode: \"%s\".\n", linInterpMode.c_str());
        return 0;
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
    std::lock_guard<std::mutex> lock(mtx);
    lastReceivedTarget = target;
}

double LinearInterpolationBuffer::getLastTarget() const
{
    std::lock_guard<std::mutex> lock(mtx);
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

void LinearInterpolationBuffer::configureBufferSize(uint8_t * msg)
{
    std::memcpy(msg + 4, &bufferSize, 2);
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
        return 0;
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

void PtBuffer::configureSubMode(uint8_t * msg)
{
    uint16_t submode = 0;
    std::memcpy(msg + 4, &submode, 2);
}

void PtBuffer::configureMessage(uint8_t * msg)
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

    mtx.lock();
    double _lastReceivedTarget = lastReceivedTarget;
    mtx.unlock();

    if (std::abs(_lastReceivedTarget - lastSentTarget) > maxDistance)
    {
        CD_WARNING("Max velocity exceeded, clipping travelled distance.\n");
        _lastReceivedTarget = lastSentTarget + maxDistance * (_lastReceivedTarget >= lastSentTarget ? 1 : -1);
        CD_INFO("New ref: %f.\n", _lastReceivedTarget);
    }

    double p = _lastReceivedTarget;
    int t = periodMs;

    int32_t position = p * factor;
    std::memcpy(msg, &position, 4);

    int16_t time = t;
    std::memcpy(msg + 4, &time, 2);

    uint8_t ic = (integrityCounter++) << 1;
    std::memcpy(msg + 7, &ic, 1);

    CD_DEBUG("Sending p %f t %d (ic %d).\n", p, t, ic >> 1);

    lastSentTarget = _lastReceivedTarget;
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

void PvtBuffer::configureSubMode(uint8_t * msg)
{
    uint16_t submode = -1;
    std::memcpy(msg + 4, &submode, 2);
}


void PvtBuffer::configureMessage(uint8_t * msg)
{
    //*************************************************************
    //-- 13. Send the 1 st PT point.
    //-- Position = 88 IU (0x000058) 1IU = 1 encoder pulse
    //-- Velocity = 3.33 IU (0x000354) 1IU = 1 encoder pulse/ 1 control loop
    //-- Time = 55 IU (0x37) 1IU = 1 control loop = 1ms by default
    //-- IC = 0 (0x00) IC=Integrity Counter
    //-- Send the following message:
    //uint8_t ptpoint1[]={0x58,0x00,0x54,0x00,0x03,0x00,0x37,0x00};

    mtx.lock();
    double currentTarget = lastReceivedTarget;
    mtx.unlock();

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

    int32_t position = p * factor;
    int16_t positionLSB = (int32_t)(position << 16) >> 16;
    int8_t positionMSB = (int32_t)(position << 8) >> 24;
    std::memcpy(msg, &positionLSB, 2);
    std::memcpy(msg + 3, &positionMSB, 1);

    double velocity = v * factor * 0.001;
    int16_t velocityInt, velocityFrac;
    TechnosoftIpos::encodeFixedPoint(velocity, &velocityInt, &velocityFrac);
    std::memcpy(msg + 2, &velocityFrac, 1);
    std::memcpy(msg + 4, &velocityInt, 2);

    int16_t time = (int16_t)(t << 7) >> 7;
    uint8_t ic = (integrityCounter++) << 1;
    uint16_t timeAndIc = time + (ic << 8);
    std::memcpy(msg + 6, &timeAndIc, 2);

    CD_DEBUG("Sending p %f v %f t %d (ic %d).\n", p, v, t, ic >> 1);

    lastSentTarget = previousTarget;
    previousTarget = currentTarget;
}
