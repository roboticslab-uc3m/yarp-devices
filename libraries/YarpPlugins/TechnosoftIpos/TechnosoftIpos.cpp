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

roboticslab::EncoderRead::EncoderRead(double initialPos)
{
    buffer[LAST] = buffer[NEXT_TO_LAST] = buffer[NEXT_TO_NEXT_TO_LAST] = initialPos;
}

// -----------------------------------------------------------------------------

void roboticslab::EncoderRead::update(double newPos)
{
    yarp::os::LockGuard guard(mutex);
    buffer[NEXT_TO_NEXT_TO_LAST] = buffer[NEXT_TO_LAST];
    buffer[NEXT_TO_LAST] = buffer[LAST];
    buffer[LAST] = newPos;
}

// -----------------------------------------------------------------------------

double roboticslab::EncoderRead::queryPosition() const
{
    yarp::os::LockGuard guard(mutex);
    return buffer[LAST];
}

// -----------------------------------------------------------------------------

double roboticslab::EncoderRead::querySpeed(double dt) const
{
    yarp::os::LockGuard guard(mutex);
    return (buffer[LAST] - buffer[NEXT_TO_LAST]) / dt;
}

// -----------------------------------------------------------------------------

double roboticslab::EncoderRead::queryAcceleration(double dt) const
{
    yarp::os::LockGuard guard(mutex);
    double lastVel = (buffer[LAST] - buffer[NEXT_TO_LAST]) / dt;
    double nextToLastVel = (buffer[NEXT_TO_LAST] - buffer[NEXT_TO_NEXT_TO_LAST]) / dt;
    return (lastVel - nextToLastVel) / dt;
}

// -----------------------------------------------------------------------------
