// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "Wiimote.hpp"

#include <yarp/conf/version.h>

#include <yarp/os/LogStream.h>

#include "LogComponent.hpp"

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue Wiimote::getAxisCount(std::size_t & axis_count)
#else
bool Wiimote::getAxisCount(unsigned int & axis_count)
#endif
{
    axis_count = 3;
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue::return_code::return_value_ok;
#else
    return true;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue Wiimote::getButtonCount(std::size_t & button_count)
#else
bool Wiimote::getButtonCount(unsigned int & button_count)
#endif
{
    button_count = 4;
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue::return_code::return_value_ok;
#else
    return true;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue Wiimote::getTrackballCount(std::size_t & trackball_count)
#else
bool Wiimote::getTrackballCount(unsigned int & trackball_count)
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
yarp::dev::ReturnValue Wiimote::getHatCount(std::size_t & hat_count)
#else
bool Wiimote::getHatCount(unsigned int & hat_count)
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
yarp::dev::ReturnValue Wiimote::getTouchSurfaceCount(std::size_t & touch_count)
#else
bool Wiimote::getTouchSurfaceCount(unsigned int & touch_count)
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
yarp::dev::ReturnValue Wiimote::getStickCount(std::size_t & stick_count)
#else
bool Wiimote::getStickCount(unsigned int & stick_count)
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
bool Wiimote::getStickDoF(unsigned int stick_id, unsigned int & DoF)
{
    DoF = 0;
    return true;
}
#endif

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue Wiimote::getButton(std::size_t button_id, double & value)
#else
bool Wiimote::getButton(unsigned int button_id, float & value)
#endif
{
    if (button_id >= 4)
    {
        yCError(WII) << "Invalid button ID:" << button_id;
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
        return yarp::dev::ReturnValue::return_code::return_value_error_input_out_of_bounds;
#else
        return false;
#endif
    }

    const auto eventData = dispatcherThread.getEventData();

    switch (button_id)
    {
    case 0:
        value = eventData.buttonA ? 1.0 : 0.0;
        break;
    case 1:
        value = eventData.buttonB ? 1.0 : 0.0;
        break;
    case 2:
        value = eventData.button1 ? 1.0 : 0.0;
        break;
    case 3:
        value = eventData.button2 ? 1.0 : 0.0;
        break;
    }

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue::return_code::return_value_ok;
#else
    return true;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue Wiimote::getTrackball(std::size_t trackball_id, yarp::dev::TrackballData & value)
#else
bool Wiimote::getTrackball(unsigned int trackball_id, yarp::sig::Vector & value)
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
yarp::dev::ReturnValue Wiimote::getHat(std::size_t hat_id, unsigned char & value)
#else
bool Wiimote::getHat(unsigned int hat_id, unsigned char & value)
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
yarp::dev::ReturnValue Wiimote::getAxis(std::size_t axis_id, double & value)
#else
bool Wiimote::getAxis(unsigned int axis_id, double & value)
#endif
{
    if (axis_id < 0 || axis_id >= 3)
    {
        yCError(WII) << "Invalid axis ID:" << axis_id;
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
        return yarp::dev::ReturnValue::return_code::return_value_error_input_out_of_bounds;
#else
        return false;
#endif
    }

    const auto eventData = dispatcherThread.getEventData();

    const int * accel;
    const int * calibOne;
    const int * calibZero;

    switch (axis_id)
    {
    case 0: // X-axis
        accel = &eventData.accelX;
        calibOne = &m_calibOneX;
        calibZero = &m_calibZeroX;
        break;
    case 1: // Y-axis
        accel = &eventData.accelY;
        calibOne = &m_calibOneY;
        calibZero = &m_calibZeroY;
        break;
    case 2: // Z-axis
        accel = &eventData.accelZ;
        calibOne = &m_calibOneZ;
        calibZero = &m_calibZeroZ;
        break;
    }

    value = static_cast<double>(*accel - *calibZero) / static_cast<double>(*calibOne - *calibZero);
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue::return_code::return_value_ok;
#else
    return true;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue Wiimote::getAllAxes(std::vector<double> & values)
{
    const auto eventData = dispatcherThread.getEventData();

    const auto accelX = &eventData.accelX;
    const auto calibOneX = &m_calibOneX;
    const auto calibZeroX = &m_calibZeroX;

    const auto accelY = &eventData.accelY;
    const auto calibOneY = &m_calibOneY;
    const auto calibZeroY = &m_calibZeroY;

    const auto accelZ = &eventData.accelZ;
    const auto calibOneZ = &m_calibOneZ;
    const auto calibZeroZ = &m_calibZeroZ;

    values = {
        static_cast<double>(*accelX - *calibZeroX) / static_cast<double>(*calibOneX - *calibZeroX),
        static_cast<double>(*accelY - *calibZeroY) / static_cast<double>(*calibOneY - *calibZeroY),
        static_cast<double>(*accelZ - *calibZeroZ) / static_cast<double>(*calibOneZ - *calibZeroZ)
    };

    return yarp::dev::ReturnValue::return_code::return_value_ok;
}
#endif

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue Wiimote::getStick(std::size_t stick_id, yarp::dev::StickData & value, yarp::dev::IJoypadController::JoypadCtrl_coordinateMode coordinate_mode)
#else
bool Wiimote::getStick(unsigned int stick_id, yarp::sig::Vector & value, yarp::dev::IJoypadController::JoypadCtrl_coordinateMode coordinate_mode)
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
yarp::dev::ReturnValue Wiimote::getTouch(std::size_t touch_id, std::vector<yarp::dev::TouchData> & value)
#else
bool Wiimote::getTouch(unsigned int touch_id, yarp::sig::Vector & value)
#endif
{
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue::return_code::return_value_error_not_implemented_by_device;
#else
    return false;
#endif
}

// -----------------------------------------------------------------------------
