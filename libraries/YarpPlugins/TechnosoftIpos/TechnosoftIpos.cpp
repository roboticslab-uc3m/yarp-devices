// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TechnosoftIpos.hpp"

#include <cstring>

#include <yarp/os/LockGuard.h>

// -----------------------------------------------------------------------------

std::string roboticslab::TechnosoftIpos::msgToStr(const yarp::dev::CanMessage & message)
{
    std::stringstream tmp;
    for(int i=0; i < message.getLen()-1; i++)
    {
        tmp << std::hex << static_cast<int>(message.getData()[i]) << " ";
    }
    tmp << std::hex << static_cast<int>(message.getData()[message.getLen()-1]);
    tmp << ". canId(";
    tmp << std::dec << (message.getId() & 0x7F);
    tmp << ") via(";
    tmp << std::hex << (message.getId() & 0xFF80);
    tmp << ").";
    return tmp.str();
}

// -----------------------------------------------------------------------------

std::string roboticslab::TechnosoftIpos::msgToStr(uint32_t cob, uint16_t len, uint8_t * msgData)
{
    std::stringstream tmp;
    for(int i=0; i < len-1; i++)
    {
        tmp << std::hex << static_cast<int>(*(msgData+i)) << " ";
    }
    tmp << std::hex << static_cast<int>(*(msgData+len-1));
    tmp << ". canId(";
    tmp << std::dec << canId;
    tmp << ") via(";
    tmp << std::hex << cob;
    tmp << ").";
    return tmp.str();
}

// -----------------------------------------------------------------------------

bool roboticslab::TechnosoftIpos::send(uint32_t cob, uint16_t len, uint8_t * msgData)
{
    canBufferSemaphore.wait();

    if ( (lastUsage - yarp::os::Time::now()) < DELAY )
        yarp::os::Time::delay( lastUsage + DELAY - yarp::os::Time::now() );

    yarp::dev::CanMessage &msg = canOutputBuffer[0];
    msg.setId(cob + canId);
    msg.setLen(len);
    std::memcpy(msg.getData(), msgData, len * sizeof(uint8_t));

    unsigned int sent;

    if( ! canDevicePtr->canWrite(canOutputBuffer, 1, &sent, true) || sent == 0 )
        return false;

    lastUsage = yarp::os::Time::now();
    canBufferSemaphore.post();
    return true;
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
        CD_ERROR("Could not send \"startPT\". %s\n", msgToStr(0x200, 2, startPT).c_str());
        return false;
    }

    CD_SUCCESS("Sent \"startPT\". %s\n", msgToStr(0x200, 2, startPT).c_str());

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
    yarp::os::LockGuard guard(mutex);

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
    yarp::os::LockGuard guard(mutex);
    return lastPosition;
}

// -----------------------------------------------------------------------------

double roboticslab::EncoderRead::querySpeed() const
{
    yarp::os::LockGuard guard(mutex);
    return lastSpeed;
}

// -----------------------------------------------------------------------------

double roboticslab::EncoderRead::queryAcceleration() const
{
    yarp::os::LockGuard guard(mutex);
    return lastAcceleration;
}

// -----------------------------------------------------------------------------

double roboticslab::EncoderRead::queryTime() const
{
    yarp::os::LockGuard guard(mutex);
    return lastStamp.getTime();
}

// -----------------------------------------------------------------------------
