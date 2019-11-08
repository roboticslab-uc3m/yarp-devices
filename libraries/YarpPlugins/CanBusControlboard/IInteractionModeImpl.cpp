// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusControlboard.hpp"

#include <yarp/os/Vocab.h>

#include <ColorDebug.h>

using namespace roboticslab;

using raw_t = yarp::dev::IInteractionModeRaw;
using enum_t = yarp::dev::InteractionModeEnum;
using multi_mapping_fn = bool (raw_t::*)(int, int *, enum_t *);

// -----------------------------------------------------------------------------

bool CanBusControlboard::getInteractionMode(int axis, yarp::dev::InteractionModeEnum * mode)
{
    CD_DEBUG("(%d)\n", axis);
    CHECK_JOINT(axis);
    return deviceMapper.mapSingleJoint(&yarp::dev::IInteractionModeRaw::getInteractionModeRaw, axis, mode);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::getInteractionModes(yarp::dev::InteractionModeEnum * modes)
{
    //CD_DEBUG("\n"); // too verbose in controlboardwrapper2 stream
    return deviceMapper.mapAllJoints(&yarp::dev::IInteractionModeRaw::getInteractionModesRaw, modes);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::getInteractionModes(int n_joints, int * joints, yarp::dev::InteractionModeEnum * modes)
{
    CD_DEBUG("\n");

    auto task = deviceMapper.createTask();
    const int * c_joints = const_cast<const int *>(joints); // workaround
    auto devices = deviceMapper.getDevices(n_joints, c_joints); // extend lifetime of local joint vector
    bool ok = true;

    for (const auto & t : devices)
    {
        auto * p = std::get<0>(t)->getHandle<yarp::dev::IInteractionModeRaw>();
        int * temp = const_cast<int *>(std::get<1>(t).data()); // workaround
        multi_mapping_fn fn = &raw_t::getInteractionModesRaw;
        ok &= p ? p->getInteractionModesRaw(std::get<1>(t).size(), temp, modes + std::get<2>(t)) : false;
    }

    return ok && task->dispatch();
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::setInteractionMode(int axis, yarp::dev::InteractionModeEnum mode)
{
    CD_DEBUG("(%d, %s)\n", axis, yarp::os::Vocab::decode(mode).c_str());
    CHECK_JOINT(axis);
    return deviceMapper.mapSingleJoint(&yarp::dev::IInteractionModeRaw::setInteractionModeRaw, axis, mode);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::setInteractionModes(yarp::dev::InteractionModeEnum * modes)
{
    CD_DEBUG("\n");
    return deviceMapper.mapAllJoints(&yarp::dev::IInteractionModeRaw::setInteractionModesRaw, modes);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::setInteractionModes(int n_joints, int * joints, yarp::dev::InteractionModeEnum * modes)
{
    CD_DEBUG("\n");

    auto task = deviceMapper.createTask();
    const int * c_joints = const_cast<const int *>(joints); // workaround
    auto devices = deviceMapper.getDevices(n_joints, c_joints); // extend lifetime of local joint vector
    bool ok = true;

    for (const auto & t : devices)
    {
        auto * p = std::get<0>(t)->getHandle<yarp::dev::IInteractionModeRaw>();
        int * temp = const_cast<int *>(std::get<1>(t).data()); // workaround
        multi_mapping_fn fn = &raw_t::getInteractionModesRaw;
        ok &= p ? task->add(p, fn, std::get<1>(t).size(), temp, modes + std::get<2>(t)), true : false;
    }

    return ok && task->dispatch();
}

// -----------------------------------------------------------------------------
