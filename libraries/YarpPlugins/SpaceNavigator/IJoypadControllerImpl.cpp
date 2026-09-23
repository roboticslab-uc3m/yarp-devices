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

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue getAxisCount(std::size_t & axis_count)
#else
bool SpaceNavigator::getAxisCount(unsigned int & axis_count)
#endif
{
    axis_count = 6;
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue::return_code::return_value_ok;
#else
    return true;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue SpaceNavigator::getButtonCount(std::size_t & button_count)
#else
bool SpaceNavigator::getButtonCount(unsigned int & button_count)
#endif
{
    button_count = 2; // button1 and button2
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue::return_code::return_value_ok;
#else
    return true;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue SpaceNavigator::getTrackballCount(std::size_t & trackball_count)
#else
bool SpaceNavigator::getTrackballCount(unsigned int & trackball_count)
#endif
{
    trackball_count = 0;
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue::return_code::return_value_ok;
#else
    return true;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue SpaceNavigator::getHatCount(std::size_t & hat_count)
#else
bool SpaceNavigator::getHatCount(unsigned int & hat_count)
#endif
{
    hat_count = 0;
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue::return_code::return_value_ok;
#else
    return true;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue SpaceNavigator::getTouchSurfaceCount(std::size_t & touch_count)
#else
bool SpaceNavigator::getTouchSurfaceCount(unsigned int & touch_count)
#endif
{
    touch_count = 0;
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue::return_code::return_value_ok;
#else
    return true;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue SpaceNavigator::getStickCount(std::size_t & stick_count)
#else
bool SpaceNavigator::getStickCount(unsigned int & stick_count)
#endif
{
    stick_count = 0;
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue::return_code::return_value_ok;
#else
    return true;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(<, 4, 0, 0)
bool SpaceNavigator::getStickDoF(unsigned int stick_id, unsigned int & DoF)
{
    return false;
}
#endif

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue SpaceNavigator::getButton(std::size_t button_id, double & value)
#else
bool SpaceNavigator::getButton(unsigned int button_id, float & value)
#endif
{
    switch (button_id)
    {
    case 0:
    {
        std::lock_guard lock(mtx);
        value = button1;
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
        return yarp::dev::ReturnValue::return_code::return_value_ok;
#else
        return true;
#endif
    }
    case 1:
    {
        std::lock_guard lock(mtx);
        value = button2;
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
        return yarp::dev::ReturnValue::return_code::return_value_ok;
#else
        return true;
#endif
    }
    default:
        yCError(SPNAV) << "Invalid button ID:" << button_id;
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
        return yarp::dev::ReturnValue::return_code::return_value_error_input_out_of_bounds;
#else
        return false;
#endif
    }
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue SpaceNavigator::getTrackball(std::size_t trackball_id, yarp::dev::TrackballData & value)
#else
bool SpaceNavigator::getTrackball(unsigned int trackball_id, yarp::sig::Vector & value)
#endif
{

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue::return_code::return_value_error_not_implemented_by_device;
#else
    return false;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue SpaceNavigator::getHat(std::size_t hat_id, unsigned char & value)
#else
bool SpaceNavigator::getHat(unsigned int hat_id, unsigned char & value)
#endif
{
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue::return_code::return_value_error_not_implemented_by_device;
#else
    return false;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue SpaceNavigator::getAxis(std::size_t axis_id, double & value)
#else
bool SpaceNavigator::getAxis(unsigned int axis_id, double & value)
#endif
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
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
        return yarp::dev::ReturnValue::return_code::return_value_error_input_out_of_bounds;
#else
        return false;
#endif
    }

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue::return_code::return_value_ok;
#else
    return true;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue SpaceNavigator::getAllAxes(std::vector<double> & values)
{
    values = {
        normalize(dx / m_fullScaleX, deadband),
        normalize(dy / m_fullScaleY, deadband),
        normalize(dz / m_fullScaleZ, deadband),
        normalize(drx / m_fullScaleRX, deadband),
        normalize(dry / m_fullScaleRY, deadband),
        normalize(drz / m_fullScaleRZ, deadband)
    };

    return yarp::dev::ReturnValue::return_code::return_value_ok;
}
#endif

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue SpaceNavigator::getStick(std::size_t stick_id, yarp::dev::StickData & value, yarp::dev::IJoypadController::JoypadCtrl_coordinateMode coordinate_mode)
#else
bool SpaceNavigator::getStick(unsigned int stick_id, yarp::sig::Vector & value, yarp::dev::IJoypadController::JoypadCtrl_coordinateMode coordinate_mode)
#endif
{
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue::return_code::return_value_error_not_implemented_by_device;
#else
    return false;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue SpaceNavigator::getTouch(std::size_t touch_id, std::vector<yarp::dev::TouchData> & value)
#else
bool SpaceNavigator::getTouch(unsigned int touch_id, yarp::sig::Vector & value)
#endif
{
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue::return_code::return_value_error_not_implemented_by_device;
#else
    return false;
#endif
}

// -----------------------------------------------------------------------------
