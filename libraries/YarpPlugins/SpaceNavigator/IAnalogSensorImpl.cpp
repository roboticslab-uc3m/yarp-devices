// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "SpaceNavigator.hpp"

#include <cmath> // std::abs, std::copysign

#include <algorithm> // std::clamp

// -----------------------------------------------------------------------------

namespace
{
    double normalize(double value, double deadband)
    {
        static constexpr auto RANGE = 1.0;

        if (std::abs(value) <= deadband)
        {
            return 0.0;
        }
        else
        {
            const double slope = RANGE / (RANGE - deadband);
            const double clamped = std::clamp(value, -RANGE, RANGE);
            return slope * std::copysign(std::abs(clamped) - deadband, value);
        }
    }
}

// -----------------------------------------------------------------------------

int SpaceNavigator::read(yarp::sig::Vector &out)
{
    spnav_event sev;

    if (spnav_poll_event(&sev) != 0)
    {
        if (sev.type == SPNAV_EVENT_MOTION)
        {
            dx = sev.motion.z;
            dy = - sev.motion.x;
            dz = sev.motion.y;
            droll = sev.motion.rz;
            dpitch = - sev.motion.rx;
            dyaw = sev.motion.ry;
        }
        else if (sev.type == SPNAV_EVENT_BUTTON)
        {
            int *p_button = (sev.button.bnum == 0) ? &button1 : &button2;
            *p_button = sev.button.press; // 1: press flank, 0: release flank
        }

        noDataCounter = 0;
    }
    else
    {
        noDataCounter++;
    }

    if (noDataCounter >= m_maxNoDataIterations)
    {
        noDataCounter = 0;
        dx = dy = dz = droll = dpitch = dyaw = 0.0;
        //button1 = button2 = 0; // buttons should preserve pressed/released state
    }

    out = {
        normalize(dx / m_fullScaleX, deadband),
        normalize(dy / m_fullScaleY, deadband),
        normalize(dz / m_fullScaleZ, deadband),
        normalize(droll / m_fullScaleRoll, deadband),
        normalize(dpitch / m_fullScalePitch, deadband),
        normalize(dyaw / m_fullScaleYaw, deadband),
        static_cast<double>(button1),
        static_cast<double>(button2)
    };

    return yarp::dev::IAnalogSensor::AS_OK;
}

// -----------------------------------------------------------------------------

int SpaceNavigator::getState(int ch)
{
    return yarp::dev::IAnalogSensor::AS_OK;
}

// -----------------------------------------------------------------------------

int SpaceNavigator::getChannels()
{
    return 8;
}

// -----------------------------------------------------------------------------

int SpaceNavigator::calibrateSensor()
{
    return yarp::dev::IAnalogSensor::AS_OK;
}

// -----------------------------------------------------------------------------

int SpaceNavigator::calibrateSensor(const yarp::sig::Vector& value)
{
    return yarp::dev::IAnalogSensor::AS_OK;
}

// -----------------------------------------------------------------------------

int SpaceNavigator::calibrateChannel(int ch)
{
    return yarp::dev::IAnalogSensor::AS_OK;
}

// -----------------------------------------------------------------------------

int SpaceNavigator::calibrateChannel(int ch, double value)
{
    return yarp::dev::IAnalogSensor::AS_OK;
}

// -----------------------------------------------------------------------------
