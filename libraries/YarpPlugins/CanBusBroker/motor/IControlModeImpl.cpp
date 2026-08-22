// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusBroker.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue CanBusBroker::getControlMode(int j, yarp::dev::ControlModeEnum & mode)
#else
bool CanBusBroker::getControlMode(int j, int * mode)
#endif
{
    CHECK_JOINT(j);
    return deviceMapper.mapSingleJoint(&yarp::dev::IControlModeRaw::getControlModeRaw, j, mode);
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue CanBusBroker::getControlModes(std::vector<yarp::dev::ControlModeEnum> & modes)
#else
bool CanBusBroker::getControlModes(int * modes)
#endif
{
    return deviceMapper.mapAllJoints(&yarp::dev::IControlModeRaw::getControlModesRaw, modes);
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue CanBusBroker::getControlModes(const std::vector<int> & joints, std::vector<yarp::dev::ControlModeEnum> & modes)
{
    return deviceMapper.mapJointGroup(&yarp::dev::IControlModeRaw::getControlModesRaw, joints, modes);
}
#else
bool CanBusBroker::getControlModes(int n_joint, const int * joints, int * modes)
{
    return deviceMapper.mapJointGroup(&yarp::dev::IControlModeRaw::getControlModesRaw, n_joint, joints, modes);
}
#endif

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue CanBusBroker::setControlMode(int j, yarp::dev::SelectableControlModeEnum mode)
#else
bool CanBusBroker::setControlMode(int j, int mode)
#endif
{
    CHECK_JOINT(j);
    return deviceMapper.mapSingleJoint(&yarp::dev::IControlModeRaw::setControlModeRaw, j, mode);
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue CanBusBroker::setControlModes(const std::vector<yarp::dev::SelectableControlModeEnum> & modes)
#else
bool CanBusBroker::setControlModes(int * modes)
#endif
{
    return deviceMapper.mapAllJoints(&yarp::dev::IControlModeRaw::setControlModesRaw, modes);
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue CanBusBroker::setControlModes(const std::vector<int> & joints, const std::vector<yarp::dev::SelectableControlModeEnum> & modes)
{
    return deviceMapper.mapJointGroup(&yarp::dev::IControlModeRaw::setControlModesRaw, joints, modes);
}
#else
bool CanBusBroker::setControlModes(int n_joint, const int * joints, int * modes)
{
    return deviceMapper.mapJointGroup(&yarp::dev::IControlModeRaw::setControlModesRaw, n_joint, joints, modes);
}
#endif

// -----------------------------------------------------------------------------
