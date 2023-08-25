// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusControlBoard.hpp"

#include <yarp/os/Log.h>
#include <yarp/os/Vocab.h>

#include "LogComponent.hpp"

using namespace roboticslab;

using raw_t = yarp::dev::IInteractionModeRaw;
using enum_t = yarp::dev::InteractionModeEnum;
using multi_mapping_fn = bool (raw_t::*)(int, int *, enum_t *);

// -----------------------------------------------------------------------------

bool CanBusControlBoard::getInteractionMode(int axis, yarp::dev::InteractionModeEnum * mode)
{
    yCTrace(CBCB, "%d", axis);
    CHECK_JOINT(axis);
    return deviceMapper.mapSingleJoint(&yarp::dev::IInteractionModeRaw::getInteractionModeRaw, axis, mode);
}

// -----------------------------------------------------------------------------

bool CanBusControlBoard::getInteractionModes(yarp::dev::InteractionModeEnum * modes)
{
    yCTrace(CBCB, "");
    return deviceMapper.mapAllJoints(&yarp::dev::IInteractionModeRaw::getInteractionModesRaw, modes);
}

// -----------------------------------------------------------------------------

bool CanBusControlBoard::getInteractionModes(int n_joints, int * joints, yarp::dev::InteractionModeEnum * modes)
{
    yCTrace(CBCB, "%d", n_joints);

    auto task = deviceMapper.createTask();
    const int * c_joints = const_cast<const int *>(joints); // workaround
    auto devices = deviceMapper.getDevices(n_joints, c_joints); // extend lifetime of local joint vector
    bool ok = true;

    for (const auto & t : devices)
    {
        auto * p = std::get<0>(t)->getHandle<yarp::dev::IInteractionModeRaw>();
        int * temp = const_cast<int *>(std::get<1>(t).data()); // workaround
        multi_mapping_fn fn = &raw_t::getInteractionModesRaw;
        ok &= p && (task->add(p, fn, std::get<1>(t).size(), temp, modes + std::get<2>(t)), true);
    }

    return ok && task->dispatch();
}

// -----------------------------------------------------------------------------

bool CanBusControlBoard::setInteractionMode(int axis, yarp::dev::InteractionModeEnum mode)
{
    yCTrace(CBCB, "%d %s", axis, yarp::os::Vocab32::decode(mode).c_str());
    CHECK_JOINT(axis);
    return deviceMapper.mapSingleJoint(&yarp::dev::IInteractionModeRaw::setInteractionModeRaw, axis, mode);
}

// -----------------------------------------------------------------------------

bool CanBusControlBoard::setInteractionModes(yarp::dev::InteractionModeEnum * modes)
{
    yCTrace(CBCB, "");
    return deviceMapper.mapAllJoints(&yarp::dev::IInteractionModeRaw::setInteractionModesRaw, modes);
}

// -----------------------------------------------------------------------------

bool CanBusControlBoard::setInteractionModes(int n_joints, int * joints, yarp::dev::InteractionModeEnum * modes)
{
    yCTrace(CBCB, "%d", n_joints);

    auto task = deviceMapper.createTask();
    const int * c_joints = const_cast<const int *>(joints); // workaround
    auto devices = deviceMapper.getDevices(n_joints, c_joints); // extend lifetime of local joint vector
    bool ok = true;

    for (const auto & t : devices)
    {
        auto * p = std::get<0>(t)->getHandle<yarp::dev::IInteractionModeRaw>();
        int * temp = const_cast<int *>(std::get<1>(t).data()); // workaround
        multi_mapping_fn fn = &raw_t::getInteractionModesRaw;
        ok &= p && (task->add(p, fn, std::get<1>(t).size(), temp, modes + std::get<2>(t)), true);
    }

    return ok && task->dispatch();
}

// -----------------------------------------------------------------------------
