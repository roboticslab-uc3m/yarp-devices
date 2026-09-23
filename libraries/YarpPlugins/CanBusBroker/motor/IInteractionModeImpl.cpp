// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusBroker.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue CanBusBroker::getInteractionMode(int axis, yarp::dev::InteractionModeEnum & mode)
#else
bool CanBusBroker::getInteractionMode(int axis, yarp::dev::InteractionModeEnum * mode)
#endif
{
    CHECK_JOINT(axis);
    return deviceMapper.mapSingleJoint(&yarp::dev::IInteractionModeRaw::getInteractionModeRaw, axis, mode);
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue CanBusBroker::getInteractionModes(std::vector<yarp::dev::InteractionModeEnum> & modes)
#else
bool CanBusBroker::getInteractionModes(yarp::dev::InteractionModeEnum * modes)
#endif
{
    return deviceMapper.mapAllJoints(&yarp::dev::IInteractionModeRaw::getInteractionModesRaw, modes);
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue CanBusBroker::getInteractionModes(const std::vector<int> & joints, std::vector<yarp::dev::InteractionModeEnum> & modes)
{
    return deviceMapper.mapJointGroup(&yarp::dev::IInteractionModeRaw::getInteractionModesRaw, joints, modes);
}
#else
bool CanBusBroker::getInteractionModes(int n_joints, int * joints, yarp::dev::InteractionModeEnum * modes)
{
    using multi_joints_fn = bool (yarp::dev::IInteractionModeRaw::*)(int, int *, yarp::dev::InteractionModeEnum *);

    auto task = deviceMapper.createTask();
    const int * c_joints = const_cast<const int *>(joints); // workaround
    auto devices = deviceMapper.getMotorDevicesWithIndices(n_joints, c_joints); // extend lifetime of local joint vector
    bool ok = true;

    for (const auto & t : devices)
    {
        auto * p = std::get<0>(t)->getHandle<yarp::dev::IInteractionModeRaw>();
        int * temp = const_cast<int *>(std::get<1>(t).data()); // workaround
        multi_joints_fn fn = &yarp::dev::IInteractionModeRaw::getInteractionModesRaw;
        ok &= p && (task->add(p, fn, std::get<1>(t).size(), temp, modes + std::get<2>(t)), true);
    }

    return ok && task->dispatch();
}
#endif

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue CanBusBroker::setInteractionMode(int axis, yarp::dev::InteractionModeEnum mode)
#else
bool CanBusBroker::setInteractionMode(int axis, yarp::dev::InteractionModeEnum mode)
#endif
{
    CHECK_JOINT(axis);
    return deviceMapper.mapSingleJoint(&yarp::dev::IInteractionModeRaw::setInteractionModeRaw, axis, mode);
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue CanBusBroker::setInteractionModes(const std::vector<yarp::dev::InteractionModeEnum> & modes)
#else
bool CanBusBroker::setInteractionModes(yarp::dev::InteractionModeEnum * modes)
#endif
{
    return deviceMapper.mapAllJoints(&yarp::dev::IInteractionModeRaw::setInteractionModesRaw, modes);
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue CanBusBroker::setInteractionModes(const std::vector<int> & joints, const std::vector<yarp::dev::InteractionModeEnum> & modes)
{
    return deviceMapper.mapJointGroup(&yarp::dev::IInteractionModeRaw::setInteractionModesRaw, joints, modes);
}
#else
bool CanBusBroker::setInteractionModes(int n_joints, int * joints, yarp::dev::InteractionModeEnum * modes)
{
    using multi_joints_fn = bool (yarp::dev::IInteractionModeRaw::*)(int, int *, yarp::dev::InteractionModeEnum *);

    auto task = deviceMapper.createTask();
    const int * c_joints = const_cast<const int *>(joints); // workaround
    auto devices = deviceMapper.getMotorDevicesWithIndices(n_joints, c_joints); // extend lifetime of local joint vector
    bool ok = true;

    for (const auto & t : devices)
    {
        auto * p = std::get<0>(t)->getHandle<yarp::dev::IInteractionModeRaw>();
        int * temp = const_cast<int *>(std::get<1>(t).data()); // workaround
        multi_joints_fn fn = &yarp::dev::IInteractionModeRaw::getInteractionModesRaw;
        ok &= p && (task->add(p, fn, std::get<1>(t).size(), temp, modes + std::get<2>(t)), true);
    }

    return ok && task->dispatch();
}
#endif

// -----------------------------------------------------------------------------
