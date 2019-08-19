// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TechnosoftIpos.hpp"

// -----------------------------------------------------------------------------

bool roboticslab::TechnosoftIpos::send(uint32_t cob, uint16_t len, uint8_t * msgData)
{
    return sender->prepareMessage(message_builder(cob + canId, len, msgData));
}

// -----------------------------------------------------------------------------

bool roboticslab::TechnosoftIpos::sendLinearInterpolationTarget()
{
    uint8_t msg[8];
    linInterpBuffer->configureMessage(msg);

    if (!send(0x400, 8, msg))
    {
        CD_ERROR("Unable to send PVT point in %d.\n", canId);
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::TechnosoftIpos::sendLinearInterpolationStart()
{
    uint8_t startPT[]= {0x1F,0x00};

    if (!send(0x200, 2, startPT))
    {
        CD_ERROR("Could not send \"startPT\". %s\n", CanUtils::msgToStr(canId, 0x200, 2, startPT).c_str());
        return false;
    }

    CD_SUCCESS("Sent \"startPT\". %s\n", CanUtils::msgToStr(canId, 0x200, 2, startPT).c_str());

    return true;
}

// -----------------------------------------------------------------------------

roboticslab::EncoderRead::EncoderRead(double initialPos)
    : lastPosition(initialPos),
      nextToLastPosition(initialPos),
      lastSpeed(0.0),
      nextToLastSpeed(0.0),
      lastAcceleration(0.0)
{
    lastStamp.update();
}

// -----------------------------------------------------------------------------

void roboticslab::EncoderRead::update(double newPos, double newTime)
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

double roboticslab::EncoderRead::queryPosition() const
{
    std::lock_guard<std::mutex> guard(encoderMutex);
    return lastPosition;
}

// -----------------------------------------------------------------------------

double roboticslab::EncoderRead::querySpeed() const
{
    std::lock_guard<std::mutex> guard(encoderMutex);
    return lastSpeed;
}

// -----------------------------------------------------------------------------

double roboticslab::EncoderRead::queryAcceleration() const
{
    std::lock_guard<std::mutex> guard(encoderMutex);
    return lastAcceleration;
}

// -----------------------------------------------------------------------------

double roboticslab::EncoderRead::queryTime() const
{
    std::lock_guard<std::mutex> guard(encoderMutex);
    return lastStamp.getTime();
}

// -----------------------------------------------------------------------------
