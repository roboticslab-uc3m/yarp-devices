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
    axis_count = 0;
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
    stick_count = 1;
    return true;
}

// -----------------------------------------------------------------------------

bool SpaceNavigator::getStickDoF(unsigned int stick_id, unsigned int & DoF)
{
    if (stick_id != 0)
    {
        return false;
    }

    DoF = 6; // 3 translations + 3 rotations
    return true;
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
    return false;
}

// -----------------------------------------------------------------------------

bool SpaceNavigator::getStick(unsigned int stick_id, yarp::sig::Vector & value, yarp::dev::IJoypadController::JoypadCtrl_coordinateMode coordinate_mode)
{
    if (stick_id != 0)
    {
        yCError(SPNAV) << "Invalid stick ID:" << stick_id;
        return false;
    }

    if (coordinate_mode != yarp::dev::IJoypadController::JypCtrlcoord_CARTESIAN)
    {
        yCError(SPNAV) << "Unsupported coordinate mode, only CARTESIAN ACCEPTED";
        return false;
    }

    std::lock_guard lock(mtx);

    value = {
        normalize(dx / m_fullScaleX, deadband),
        normalize(dy / m_fullScaleY, deadband),
        normalize(dz / m_fullScaleZ, deadband),
        normalize(drx / m_fullScaleRoll, deadband),
        normalize(dry / m_fullScalePitch, deadband),
        normalize(drz / m_fullScaleYaw, deadband)
    };

    return true;
}

// -----------------------------------------------------------------------------

bool SpaceNavigator::getTouch(unsigned int touch_id, yarp::sig::Vector & value)
{
    return false;
}

// -----------------------------------------------------------------------------
