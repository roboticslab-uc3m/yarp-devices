// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "LinearInterpolationBuffer.hpp"

#include <ColorDebug.h>

using namespace roboticslab;

LinearInterpolationBuffer::LinearInterpolationBuffer(double _periodMs, TechnosoftIpos * _technosoftIpos)
    : technosoftIpos(_technosoftIpos),
      periodMs(_periodMs),
      lastSentTarget(0.0),
      lastReceivedTarget(0.0)
{}

void LinearInterpolationBuffer::setInitialReference(double target)
{
    lastSentTarget = target;
}

void LinearInterpolationBuffer::updateTarget(double target)
{
    mutex.lock();
    lastReceivedTarget = target;
    mutex.unlock();
}

PtBuffer::PtBuffer(double periodMs, TechnosoftIpos * technosoftIpos)
    : LinearInterpolationBuffer(periodMs, technosoftIpos)
{}

void PtBuffer::setSubMode(uint8_t * msg)
{
    uint16_t submode = 0;
    std::memcpy(msg + 4, &submode, 2);
}

void PtBuffer::createMessage(uint8_t * msg)
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

    mutex.lock();
    double p = lastReceivedTarget;
    lastSentTarget = lastReceivedTarget;
    mutex.unlock();

    int t = periodMs;
    const double factor = technosoftIpos->tr * (technosoftIpos->encoderPulses / 360.0);

    int32_t position = p * factor;  // Apply tr & convert units to encoder increments
    std::memcpy(msg, &position, 4);

    int16_t time = t;
    std::memcpy(msg + 4, &time, 2);

    uint8_t ic = (++technosoftIpos->pvtPointCounter) << 1;
    std::memcpy(msg + 7, &ic, 1);

    CD_DEBUG("Sending to canId %d: pos %f, time %d, ic %d.\n",
                technosoftIpos->canId, p, t, technosoftIpos->pvtPointCounter);
}

PvtBuffer::PvtBuffer(double periodMs, TechnosoftIpos * technosoftIpos)
    : LinearInterpolationBuffer(periodMs, technosoftIpos)
{}

void PvtBuffer::setSubMode(uint8_t * msg)
{
    uint16_t submode = -1;
    std::memcpy(msg + 4, &submode, 2);
}


void PvtBuffer::createMessage(uint8_t * msg)
{
    //*************************************************************
    //-- 13. Send the 1 st PT point.
    //-- Position = 88 IU (0x000058) 1IU = 1 encoder pulse
    //-- Velocity = 3.33 IU (0x000354) 1IU = 1 encoder pulse/ 1 control loop
    //-- Time = 55 IU (0x37) 1IU = 1 control loop = 1ms by default
    //-- IC = 0 (0x00) IC=Integrity Counter
    //-- Send the following message:
    //uint8_t ptpoint1[]={0x58,0x00,0x54,0x00,0x03,0x00,0x37,0x00};

    mutex.lock();
    double p = lastReceivedTarget;
    double previousTarget = lastSentTarget;
    lastSentTarget = lastReceivedTarget;
    mutex.unlock();

    double v = (lastReceivedTarget - previousTarget) / (periodMs / 1000.0);
    int t = periodMs;
    const double factor = technosoftIpos->tr * (technosoftIpos->encoderPulses / 360.0);

    int32_t position = p * factor;  // Apply tr & convert units to encoder increments
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
    uint8_t ic = (++technosoftIpos->pvtPointCounter) << 1;
    uint16_t timeAndIc = time + ic;
    std::memcpy(msg + 6, &timeAndIc, 2);

    CD_DEBUG("Sending to canId %d: pos %f, vel %f, time %d, ic %d.\n",
                technosoftIpos->canId, p, v, t, technosoftIpos->pvtPointCounter);
}
