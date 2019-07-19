// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "PvtPeriodicThread.hpp"

#include <cstring>

#include <ColorDebug.h>

using namespace roboticslab;

PvtPeriodicThread::PvtPeriodicThread(double _period)
    : yarp::os::PeriodicThread(_period),
      period(_period),
      technosoftIpos(0),
      lastTargetSent(0.0),
      lastTargetReceived(0.0)
{}

void PvtPeriodicThread::registerDriveHandle(TechnosoftIpos * technosoftIpos)
{
    this->technosoftIpos = technosoftIpos;
}

void PvtPeriodicThread::setInitialPose(double target)
{
    lastTargetSent = target;
}

void PvtPeriodicThread::updateTarget(double target)
{
    mutex.lock();
    lastTargetReceived = target;
    mutex.unlock();
}

void PvtPeriodicThread::run()
{
    uint8_t msg[8];
    PvtPoint pvtPoint;

    mutex.lock();
    pvtPoint.p = lastTargetReceived;
    double previousTarget = lastTargetSent;
    lastTargetSent = lastTargetReceived;
    mutex.unlock();

    pvtPoint.v = (lastTargetReceived - previousTarget) / period;
    pvtPoint.t = period * 1000;

    createPvtMessage(pvtPoint, msg);

    if (!technosoftIpos->send(0x400, 8, msg))
    {
        CD_WARNING("Unable to send PVT point in %d.\n", technosoftIpos->canId);
    }
    else
    {
        CD_SUCCESS("Sent to canId %d: pos %f, vel %f, time %d, ic %d.\n",
            technosoftIpos->canId, pvtPoint.p, pvtPoint.v, pvtPoint.t, technosoftIpos->pvtPointCounter);
    }
}

void PvtPeriodicThread::createPvtMessage(const PvtPoint & pvtPoint, uint8_t * msg)
{
    //*************************************************************
    //-- 13. Send the 1 st PT point.
    //-- Position = 88 IU (0x000058) 1IU = 1 encoder pulse
    //-- Velocity = 3.33 IU (0x000354) 1IU = 1 encoder pulse/ 1 control loop
    //-- Time = 55 IU (0x37) 1IU = 1 control loop = 1ms by default
    //-- IC = 0 (0x00) IC=Integrity Counter
    //-- Send the following message:
    //uint8_t ptpoint1[]={0x58,0x00,0x54,0x00,0x03,0x00,0x37,0x00};

    const double factor = technosoftIpos->tr * (technosoftIpos->encoderPulses / 360.0);

    int32_t position = pvtPoint.p * factor;  // Apply tr & convert units to encoder increments
    int16_t positionLSB = (int32_t)(position << 16) >> 16;
    int8_t positionMSB = (int32_t)(position << 8) >> 24;
    std::memcpy(msg, &positionLSB, 2);
    std::memcpy(msg + 3, &positionMSB, 1);

    double velocity = pvtPoint.v * factor * 0.001;
    int16_t velocityInt, velocityFrac;
    TechnosoftIpos::encodeFixedPoint(velocity, &velocityInt, &velocityFrac);
    std::memcpy(msg + 2, &velocityFrac, 1);
    std::memcpy(msg + 4, &velocityInt, 2);

    int16_t time = (int16_t)(pvtPoint.t << 7) >> 7;
    uint8_t ic = (++technosoftIpos->pvtPointCounter) << 1;
    uint16_t timeAndIc = time + ic;
    std::memcpy(msg + 6, &timeAndIc, 2);
}
