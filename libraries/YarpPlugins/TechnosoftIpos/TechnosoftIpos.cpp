// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TechnosoftIpos.hpp"

#include "CanUtils.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool TechnosoftIpos::sendLinearInterpolationTarget()
{
    return can->rpdo3()->write<uint64_t>(linInterpBuffer->makeDataRecord());
}

// -----------------------------------------------------------------------------

bool TechnosoftIpos::sendLinearInterpolationStart()
{
    return can->rpdo1()->write<uint16_t>(0x001F);
}

// -----------------------------------------------------------------------------

EncoderRead::EncoderRead(double initialPos)
    : lastPosition(initialPos),
      nextToLastPosition(initialPos),
      lastSpeed(0.0),
      nextToLastSpeed(0.0),
      lastAcceleration(0.0)
{
    lastStamp.update();
}

// -----------------------------------------------------------------------------

void EncoderRead::reset(double pos)
{
    std::lock_guard<std::mutex> guard(encoderMutex);
    lastPosition = nextToLastPosition = pos;
    lastSpeed = nextToLastSpeed = lastAcceleration = 0.0;
}

// -----------------------------------------------------------------------------

void EncoderRead::update(double newPos, double newTime)
{
    std::lock_guard<std::mutex> guard(encoderMutex);

    const double lastTime = lastStamp.getTime();

    nextToLastPosition = lastPosition;
    nextToLastSpeed = lastSpeed;

    if (newTime)
    {
        lastStamp.update(newTime);
    }
    else
    {
        lastStamp.update();
    }

    double dt = lastStamp.getTime() - lastTime;

    lastPosition = newPos;
    lastSpeed = (lastPosition - nextToLastPosition) / dt;
    lastAcceleration = (lastSpeed - nextToLastSpeed) / dt;
}

// -----------------------------------------------------------------------------

double EncoderRead::queryPosition() const
{
    std::lock_guard<std::mutex> guard(encoderMutex);
    return lastPosition;
}

// -----------------------------------------------------------------------------

double EncoderRead::querySpeed() const
{
    std::lock_guard<std::mutex> guard(encoderMutex);
    return lastSpeed;
}

// -----------------------------------------------------------------------------

double EncoderRead::queryAcceleration() const
{
    std::lock_guard<std::mutex> guard(encoderMutex);
    return lastAcceleration;
}

// -----------------------------------------------------------------------------

double EncoderRead::queryTime() const
{
    std::lock_guard<std::mutex> guard(encoderMutex);
    return lastStamp.getTime();
}

// -----------------------------------------------------------------------------

std::string TechnosoftIposEmcy::codeToMessage(std::uint16_t code)
{
    switch (code)
    {
    case 0x2310:
        return "Continuous over-current";
    case 0x2340:
        return "Short-circuit";
    case 0x3210:
        return "DC-link over-voltage";
    case 0x3220:
        return "DC-link under-voltage";
    case 0x4280:
        return "Over temperature motor";
    case 0x4310:
        return "Over temperature drive";
    case 0x5441:
        return "Drive disabled due to enable or STO input";
    case 0x5442:
        return "Negative limit switch active";
    case 0x5443:
        return "Positive limit switch active";
    case 0x6100: // overridden
        return "Invalid setup data";
    case 0x7300: // contains more data
        return "Sensor error";
    case 0x7500: // contains more data
        return "Communication error";
    case 0x8331:
        return "I2t protection triggered";
    case 0x8580:
        return "Position wraparound";
    case 0x8611:
        return "Control error / Following error";
    case 0x9000: // overridden
        return "Command error";
    case 0xFF01: // contains more data
        return "Generic interpolated position mode error (PVT / PT error)";
    case 0xFF02:
        return "Change set acknowledge bit wrong value";
    case 0xFF03:
        return "Specified homing method not available";
    case 0xFF04:
        return "A wrong mode is set in object 6060h , modes of operation";
    case 0xFF05:
        return "Specified digital I/O line not available";
    case 0xFF06:
        return "Positive software position limit triggered";
    case 0xFF07:
        return "Negative software position limit triggered";
    case 0xFF08:
        return "Enable circuit hardware error";
    default:
        return EmcyCodeRegistry::codeToMessage(code);
    }
}

// -----------------------------------------------------------------------------
