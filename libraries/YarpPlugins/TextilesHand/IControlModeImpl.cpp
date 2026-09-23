// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TextilesHand.hpp"

#include <yarp/os/LogStream.h>

#include "LogComponent.hpp"

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue TextilesHand::getAvailableControlModes(int j, std::vector<yarp::dev::SelectableControlModeEnum> & avail)
{
    if (j != 0) return yarp::dev::ReturnValue::return_code::return_value_error_input_out_of_bounds;
    avail.clear();
    avail.push_back(yarp::dev::SelectableControlModeEnum::VOCAB_CM_POSITION_DIRECT);
    return yarp::dev::ReturnValue::return_code::return_value_ok;
}
#endif

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue TextilesHand::getControlMode(int j, yarp::dev::ControlModeEnum & mode)
{
    if (j != 0) return yarp::dev::ReturnValue::return_code::return_value_error_input_out_of_bounds;
    mode = yarp::dev::ControlModeEnum::VOCAB_CM_POSITION_DIRECT;
    return yarp::dev::ReturnValue::return_code::return_value_ok;
}
#else
bool TextilesHand::getControlMode(int j, int * mode)
{
    if (j != 0) return false;
    *mode = VOCAB_CM_POSITION_DIRECT;
    return true;
}
#endif

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue TextilesHand::getControlModes(std::vector<yarp::dev::ControlModeEnum> & modes)
{
    return getControlMode(0, modes[0]);
}
#else
bool TextilesHand::getControlModes(int * modes)
{
    return getControlMode(0, &modes[0]);
}
#endif

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue TextilesHand::getControlModes(const std::vector<int> & joints, std::vector<yarp::dev::ControlModeEnum> & modes)
{
    return getControlMode(joints[0], modes[0]);
}
#else
bool TextilesHand::getControlModes(int n_joint, const int * joints, int * modes)
{
    return getControlMode(joints[0], &modes[0]);
}
#endif

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue TextilesHand::setControlMode(int j, yarp::dev::SelectableControlModeEnum mode)
#else
bool TextilesHand::setControlMode(int j, int mode)
#endif
{
    yCWarning(TXT) << "setControlMode() not supported";
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue::return_code::return_value_error_not_implemented_by_device;
#else
    return false;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue TextilesHand::setControlModes(const std::vector<yarp::dev::SelectableControlModeEnum> & modes)
#else
bool TextilesHand::setControlModes(int * modes)
#endif
{
    yCWarning(TXT) << "setControlModes() not supported";
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue::return_code::return_value_error_not_implemented_by_device;
#else
    return false;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue TextilesHand::setControlModes(const std::vector<int> & joints, const std::vector<yarp::dev::SelectableControlModeEnum> & modes)
#else
bool TextilesHand::setControlModes(int n_joint, const int * joints, int * modes)
#endif
{
    yCWarning(TXT) << "setControlModes() not supported";
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue::return_code::return_value_error_not_implemented_by_device;
#else
    return false;
#endif
}

// -----------------------------------------------------------------------------
