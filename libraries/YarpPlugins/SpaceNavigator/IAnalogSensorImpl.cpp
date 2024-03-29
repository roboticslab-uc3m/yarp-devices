// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "SpaceNavigator.hpp"

using namespace roboticslab;

constexpr auto NUM_CHANNELS = 8;
constexpr auto FULL_SCALE_X = 460.0;
constexpr auto FULL_SCALE_Y = 430.0;
constexpr auto FULL_SCALE_Z = 440.0;
constexpr auto FULL_SCALE_ROLL = 415.0;
constexpr auto FULL_SCALE_PITCH = 405.0;
constexpr auto FULL_SCALE_YAW = 435.0;
constexpr auto MAX_NO_DATA_ITERATIONS = 10;

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

    if (noDataCounter >= MAX_NO_DATA_ITERATIONS)
    {
        noDataCounter = 0;
        dx = dy = dz = droll = dpitch = dyaw = 0.0;
        //button1 = button2 = 0; // buttons should preserve pressed/released state
    }

    out.resize(NUM_CHANNELS);

    out[0] = enforceDeadband(enforceRange(dx / FULL_SCALE_X));
    out[1] = enforceDeadband(enforceRange(dy / FULL_SCALE_Y));
    out[2] = enforceDeadband(enforceRange(dz / FULL_SCALE_Z));
    out[3] = enforceDeadband(enforceRange(droll / FULL_SCALE_ROLL));
    out[4] = enforceDeadband(enforceRange(dpitch / FULL_SCALE_PITCH));
    out[5] = enforceDeadband(enforceRange(dyaw / FULL_SCALE_YAW));

    out[6] = button1;
    out[7] = button2;

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
    return NUM_CHANNELS;
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
