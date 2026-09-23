// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "LacqueyFetch.hpp"

#include <yarp/os/LogStream.h>

#include "LogComponent.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue LacqueyFetch::getControlModeRaw(int j, yarp::dev::ControlModeEnum & mode)
#else
bool LacqueyFetch::getControlModeRaw(int j, int * mode)
#endif
{
    CHECK_JOINT(j);
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    mode = yarp::dev::ControlModeEnum::VOCAB_CM_PWM;
    return yarp::dev::ReturnValue::return_code::return_value_ok;
#else
    *mode = VOCAB_CM_PWM;
    return true;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue LacqueyFetch::getControlModesRaw(std::vector<yarp::dev::ControlModeEnum> & modes)
#else
bool LacqueyFetch::getControlModesRaw(int * modes)
#endif
{
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return getControlModeRaw(0, modes[0]);
#else
    return getControlModeRaw(0, &modes[0]);
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue LacqueyFetch::getControlModesRaw(const std::vector<int> & joints, std::vector<yarp::dev::ControlModeEnum> & modes)
#else
bool LacqueyFetch::getControlModesRaw(int n_joint, const int * joints, int * modes)
#endif
{
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return getControlModeRaw(joints[0], modes[0]);
#else
    return getControlModeRaw(joints[0], &modes[0]);
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue LacqueyFetch::setControlModeRaw(int j, yarp::dev::SelectableControlModeEnum mode)
#else
bool LacqueyFetch::setControlModeRaw(int j, int mode)
#endif
{
    CHECK_JOINT(j);
    yCIWarning(LCQ, id()) << "setControlModeRaw() not supported";
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue::return_code::return_value_error_not_implemented_by_device;
#else
    return false;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue LacqueyFetch::setControlModesRaw(const std::vector<yarp::dev::SelectableControlModeEnum> & modes)
#else
bool LacqueyFetch::setControlModesRaw(int * modes)
#endif
{
    yCIWarning(LCQ, id()) << "setControlModesRaw() not supported";
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue::return_code::return_value_error_not_implemented_by_device;
#else
    return false;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue LacqueyFetch::setControlModesRaw(const std::vector<int> & joints, const std::vector<yarp::dev::SelectableControlModeEnum> & modes)
#else
bool LacqueyFetch::setControlModesRaw(int n_joint, const int * joints, int * modes)
#endif
{
    yCIWarning(LCQ, id()) << "setControlModesRaw() not supported";
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue::return_code::return_value_error_not_implemented_by_device;
#else
    return false;
#endif
}

// -----------------------------------------------------------------------------
