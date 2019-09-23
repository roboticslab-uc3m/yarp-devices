// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusControlboard.hpp"

#include <yarp/os/Vocab.h>

#include <ColorDebug.h>

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool CanBusControlboard::getInteractionMode(int axis, yarp::dev::InteractionModeEnum * mode)
{
    // CD_DEBUG("(%d)\n", axis);  // -- is printed too many times...
    CHECK_JOINT(axis);
    return deviceMapper.singleJointMapping(axis, mode, &yarp::dev::IInteractionModeRaw::getInteractionModeRaw);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::getInteractionModes(yarp::dev::InteractionModeEnum * modes)
{
    CD_DEBUG("\n");
    return deviceMapper.fullJointMapping(modes, &yarp::dev::IInteractionModeRaw::getInteractionModesRaw);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::getInteractionModes(int n_joints, int * joints, yarp::dev::InteractionModeEnum * modes)
{
    CD_DEBUG("\n");

    bool ok = true;
    const int * c_joints = const_cast<const int *>(joints); // workaround

    for (const auto & t : deviceMapper.getDevices(n_joints, c_joints))
    {
        yarp::dev::IInteractionModeRaw * p = std::get<0>(t)->iInteractionModeRaw;
        const auto & localIndices = deviceMapper.computeLocalIndices(std::get<1>(t), joints, std::get<2>(t));
        int * temp = const_cast<int *>(localIndices.data()); // workaround
        ok &= p ? p->getInteractionModesRaw(std::get<1>(t), temp, modes + std::get<2>(t)) : false;
    }

    return ok;
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::setInteractionMode(int axis, yarp::dev::InteractionModeEnum mode)
{
    CD_DEBUG("(%d, %s)\n", axis, yarp::os::Vocab::decode(mode).c_str());
    CHECK_JOINT(axis);
    return deviceMapper.singleJointMapping(axis, mode, &yarp::dev::IInteractionModeRaw::setInteractionModeRaw);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::setInteractionModes(yarp::dev::InteractionModeEnum * modes)
{
    CD_DEBUG("\n");
    return deviceMapper.fullJointMapping(modes, &yarp::dev::IInteractionModeRaw::setInteractionModesRaw);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::setInteractionModes(int n_joints, int * joints, yarp::dev::InteractionModeEnum * modes)
{
    CD_DEBUG("\n");

    bool ok = true;
    const int * c_joints = const_cast<const int *>(joints); // workaround

    for (const auto & t : deviceMapper.getDevices(n_joints, c_joints))
    {
        yarp::dev::IInteractionModeRaw * p = std::get<0>(t)->iInteractionModeRaw;
        const auto & localIndices = deviceMapper.computeLocalIndices(std::get<1>(t), joints, std::get<2>(t));
        int * temp = const_cast<int *>(localIndices.data()); // workaround
        ok &= p ? p->setInteractionModesRaw(std::get<1>(t), temp, modes + std::get<2>(t)) : false;
    }

    return ok;
}

// -----------------------------------------------------------------------------
