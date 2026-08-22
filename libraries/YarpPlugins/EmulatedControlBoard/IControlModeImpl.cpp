// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "EmulatedControlBoard.hpp"

#include <yarp/os/LogStream.h>
#include <yarp/os/Vocab.h>

#include "LogComponent.hpp"

// ------------------- IControlMode Related ------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue EmulatedControlBoard::getAvailableControlModes(int j, std::vector<yarp::dev::SelectableControlModeEnum> & avail)
{
    if (j < 0 || j >= m_axes)
    {
        yCError(ECB) << "Axis index out of bounds:" << j;
        return yarp::dev::ReturnValue::return_code::return_value_error_input_out_of_bounds;
    }

    avail.clear();
    avail.push_back(yarp::dev::SelectableControlModeEnum::VOCAB_CM_POSITION);
    avail.push_back(yarp::dev::SelectableControlModeEnum::VOCAB_CM_VELOCITY);
    avail.push_back(yarp::dev::SelectableControlModeEnum::VOCAB_CM_POSITION_DIRECT);

    return yarp::dev::ReturnValue::return_code::return_value_ok;
}
#endif

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue EmulatedControlBoard::getControlMode(int j, yarp::dev::ControlModeEnum & mode)
#else
bool EmulatedControlBoard::getControlMode(int j, int * mode)
#endif
{
    if (j < 0 || j >= m_axes)
    {
        yCError(ECB) << "Axis index out of bounds:" << j;
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
        return yarp::dev::ReturnValue::return_code::return_value_error_input_out_of_bounds;
#else
        return false;
#endif
    }

    switch (controlMode)
    {
    case POSITION_MODE:
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
        mode = yarp::dev::ControlModeEnum::VOCAB_CM_POSITION;
#else
        *mode = VOCAB_CM_POSITION;
#endif
        break;
    case VELOCITY_MODE:
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
        mode = yarp::dev::ControlModeEnum::VOCAB_CM_VELOCITY;
#else
        *mode = VOCAB_CM_VELOCITY;
#endif
        break;
    case POSITION_DIRECT_MODE:
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
        mode = yarp::dev::ControlModeEnum::VOCAB_CM_POSITION_DIRECT;
#else
        *mode = VOCAB_CM_POSITION_DIRECT;
#endif
        break;
    default:
        yCError(ECB) << "Currently unsupported mode:" << yarp::os::Vocab32::decode(controlMode);
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
yarp::dev::ReturnValue EmulatedControlBoard::getControlModes(std::vector<yarp::dev::ControlModeEnum> & modes)
#else
bool EmulatedControlBoard::getControlModes(int * modes)
#endif
{
    bool ok = true;

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    modes.resize(m_axes);
#endif

    for (unsigned int i = 0; i < m_axes; i++)
    {
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
        yarp::dev::ControlModeEnum mode;
        ok &= getControlMode(i, mode);
        modes[i] = mode;
#else
        ok &= getControlMode(i, &modes[i]);
#endif
    }

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return ok
        ? yarp::dev::ReturnValue::return_code::return_value_ok
        : yarp::dev::ReturnValue::return_code::return_value_error_method_failed;
#else
    return ok;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue EmulatedControlBoard::getControlModes(const std::vector<int> & joints, std::vector<yarp::dev::ControlModeEnum> & modes)
#else
bool EmulatedControlBoard::getControlModes(int n_joint, const int * joints, int * modes)
#endif
{
    bool ok = true;

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    modes.resize(joints.size());

    for (size_t i = 0; i < joints.size(); i++)
    {
        yarp::dev::ControlModeEnum mode;
        ok &= getControlMode(joints[i], mode);
        modes[i] = mode;
#else
    for (int i = 0; i < n_joint; i++)
    {
        ok &= getControlMode(joints[i], &modes[i]);
#endif
    }

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return ok
        ? yarp::dev::ReturnValue::return_code::return_value_ok
        : yarp::dev::ReturnValue::return_code::return_value_error_method_failed;
#else
    return ok;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue EmulatedControlBoard::setControlMode(int j, const yarp::dev::SelectableControlModeEnum mode)
#else
bool EmulatedControlBoard::setControlMode(int j, const int mode)
#endif
{
    if (j < 0 || j >= m_axes)
    {
        yCError(ECB) << "Axis index out of bounds:" << j;
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
        return yarp::dev::ReturnValue::return_code::return_value_error_input_out_of_bounds;
#else
        return false;
#endif
    }

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    auto ret_ok = yarp::dev::ReturnValue::return_code::return_value_ok;
    auto ret_fail = yarp::dev::ReturnValue::return_code::return_value_error_method_failed;
#else
    auto ret_ok = true;
    auto ret_fail = false;
#endif

    switch (mode)
    {
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    case yarp::dev::SelectableControlModeEnum::VOCAB_CM_POSITION:
#else
    case VOCAB_CM_POSITION:
#endif
        return (controlMode == POSITION_MODE || stop(j)) && (controlMode = POSITION_MODE, true)
            ? ret_ok
            : ret_fail;
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    case yarp::dev::SelectableControlModeEnum::VOCAB_CM_VELOCITY:
#else
    case VOCAB_CM_VELOCITY:
#endif
        return (controlMode == VELOCITY_MODE || stop(j)) && (controlMode = VELOCITY_MODE, true)
            ? ret_ok
            : ret_fail;
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    case yarp::dev::SelectableControlModeEnum::VOCAB_CM_POSITION_DIRECT:
#else
    case VOCAB_CM_POSITION_DIRECT:
#endif
        return (controlMode == POSITION_DIRECT_MODE || stop(j)) && (controlMode = POSITION_DIRECT_MODE, true)
            ? ret_ok
            : ret_fail;
    default:
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
        return yarp::dev::ReturnValue::return_code::return_value_error_input_out_of_bounds;
#else
        return false;
#endif
    }
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue EmulatedControlBoard::setControlModes(const std::vector<yarp::dev::SelectableControlModeEnum> & modes)
#else
bool EmulatedControlBoard::setControlModes(int * modes)
#endif
{
    bool ok = true;

    for (unsigned int i = 0; i < m_axes; i++)
    {
        ok &= setControlMode(i, modes[i]);
    }

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return ok
        ? yarp::dev::ReturnValue::return_code::return_value_ok
        : yarp::dev::ReturnValue::return_code::return_value_error_method_failed;
#else
    return ok;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue EmulatedControlBoard::setControlModes(const std::vector<int> & joints, const std::vector<yarp::dev::SelectableControlModeEnum> & modes)
#else
bool EmulatedControlBoard::setControlModes(int n_joint, const int * joints, int * modes)
#endif
{
    bool ok = true;

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    for (int j = 0; j < joints.size(); j++)
#else
    for (int j = 0; j < n_joint; j++)
#endif
    {
        ok &= setControlMode(joints[j], modes[j]);
    }

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return ok
        ? yarp::dev::ReturnValue::return_code::return_value_ok
        : yarp::dev::ReturnValue::return_code::return_value_error_method_failed;
#else
    return ok;
#endif
}

// -----------------------------------------------------------------------------
