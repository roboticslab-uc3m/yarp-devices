// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "SpaceNavigator.hpp"

#include <cmath> // std::abs, std::copysign

#include <algorithm> // std::clamp

#include <yarp/os/LogStream.h>

#include "LogComponent.hpp"

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

bool SpaceNavigator::getAxisCount(unsigned int & axis_count)
{
    axis_count = 6;
    return true;
}

// -----------------------------------------------------------------------------

bool SpaceNavigator::getButtonCount(unsigned int & button_count)
{
    button_count = 2; // button1 and button2
    return true;
}

// -----------------------------------------------------------------------------

bool SpaceNavigator::getTrackballCount(unsigned int & trackball_count)
{
    trackball_count = 0;
    return true;
}

// -----------------------------------------------------------------------------

bool SpaceNavigator::getHatCount(unsigned int & hat_count)
{
    hat_count = 0;
    return true;
}

// -----------------------------------------------------------------------------

bool SpaceNavigator::getTouchSurfaceCount(unsigned int & touch_count)
{
    touch_count = 0;
    return true;
}

// -----------------------------------------------------------------------------

bool SpaceNavigator::getStickCount(unsigned int & stick_count)
{
    stick_count = 0;
    return true;
}

// -----------------------------------------------------------------------------

bool SpaceNavigator::getStickDoF(unsigned int stick_id, unsigned int & DoF)
{
    return false;
}

// -----------------------------------------------------------------------------

bool SpaceNavigator::getButton(unsigned int button_id, float & value)
{
    switch (button_id)
    {
    case 0:
    {
        std::lock_guard lock(mtx);
        value = button1;
        return true;
    }
    case 1:
    {
        std::lock_guard lock(mtx);
        value = button2;
        return true;
    }
    default:
        yCError(SPNAV) << "Invalid button ID:" << button_id;
        return false;
    }
}

// -----------------------------------------------------------------------------

bool SpaceNavigator::getTrackball(unsigned int trackball_id, yarp::sig::Vector & value)
{
    return false;
}

// -----------------------------------------------------------------------------

bool SpaceNavigator::getHat(unsigned int hat_id, unsigned char & value)
{
    return false;
}

// -----------------------------------------------------------------------------

bool SpaceNavigator::getAxis(unsigned int axis_id, double & value)
{
    std::lock_guard lock(mtx);

    switch (axis_id)
    {
    case 0:
        value = normalize(dx / m_fullScaleX, deadband);
        break;
    case 1:
        value = normalize(dy / m_fullScaleY, deadband);
        break;
    case 2:
        value = normalize(dz / m_fullScaleZ, deadband);
        break;
    case 3:
        value = normalize(drx / m_fullScaleRX, deadband);
        break;
    case 4:
        value = normalize(dry / m_fullScaleRY, deadband);
        break;
    case 5:
        value = normalize(drz / m_fullScaleRZ, deadband);
        break;
    default:
        yCError(SPNAV) << "Invalid axis ID:" << axis_id;
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------------

bool SpaceNavigator::getStick(unsigned int stick_id, yarp::sig::Vector & value, yarp::dev::IJoypadController::JoypadCtrl_coordinateMode coordinate_mode)
{
    return false;
}

// -----------------------------------------------------------------------------

bool SpaceNavigator::getTouch(unsigned int touch_id, yarp::sig::Vector & value)
{
    return false;
}

// -----------------------------------------------------------------------------
