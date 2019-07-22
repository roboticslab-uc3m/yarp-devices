// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TechnosoftIpos.hpp"

#include <cstring>

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

void roboticslab::TechnosoftIpos::createPvtMessage(const PvtPoint & pvtPoint, uint8_t * msg)
{
    //*************************************************************
    //-- 13. Send the 1 st PT point.
    //-- Position = 88 IU (0x000058) 1IU = 1 encoder pulse
    //-- Velocity = 3.33 IU (0x000354) 1IU = 1 encoder pulse/ 1 control loop
    //-- Time = 55 IU (0x37) 1IU = 1 control loop = 1ms by default
    //-- IC = 0 (0x00) IC=Integrity Counter
    //-- Send the following message:
    //uint8_t ptpoint1[]={0x58,0x00,0x54,0x00,0x03,0x00,0x37,0x00};

    const double factor = tr * (encoderPulses / 360.0);

    int32_t position = pvtPoint.p * factor;  // Apply tr & convert units to encoder increments
    int16_t positionLSB = (int32_t)(position << 16) >> 16;
    int8_t positionMSB = (int32_t)(position << 8) >> 24;
    std::memcpy(msg, &positionLSB, 2);
    std::memcpy(msg + 3, &positionMSB, 1);

    double velocity = pvtPoint.v * factor * 0.001;
    int16_t velocityInt, velocityFrac;
    encodeFixedPoint(velocity, &velocityInt, &velocityFrac);
    std::memcpy(msg + 2, &velocityFrac, 1);
    std::memcpy(msg + 4, &velocityInt, 2);

    int16_t time = (int16_t)(pvtPoint.t << 7) >> 7;
    uint8_t ic = (++pvtPointCounter) << 1;
    uint16_t timeAndIc = time + ic;
    std::memcpy(msg + 6, &timeAndIc, 2);
}

// -----------------------------------------------------------------------------

bool roboticslab::TechnosoftIpos::sendPvtTarget()
{
    uint8_t msg[8];
    PvtPoint pvtPoint;

    pvtMutex.lock();
    pvtPoint.p = lastPvtTargetReceived;
    double previousTarget = lastPvtTargetSent;
    lastPvtTargetSent = lastPvtTargetReceived;
    pvtMutex.unlock();

    pvtPoint.v = (lastPvtTargetReceived - previousTarget) / (pvtModeMs / 1000.0);
    pvtPoint.t = pvtModeMs;

    createPvtMessage(pvtPoint, msg);

    if (!send(0x400, 8, msg))
    {
        CD_WARNING("Unable to send PVT point in %d.\n", canId);
    }
    else
    {
        CD_SUCCESS("Sent to canId %d: pos %f, vel %f, time %d, ic %d.\n",
            canId, pvtPoint.p, pvtPoint.v, pvtPoint.t, pvtPointCounter);
    }

    return true;
}

// -----------------------------------------------------------------------------
