// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusControlboard.hpp"

#include <yarp/os/Vocab.h>

#include <ColorDebug.h>

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool CanBusControlboard::getControlMode(int j, int * mode)
{
    //CD_DEBUG("(%d)\n", j); //-- Too verbose in controlboardwrapper2 stream
    CHECK_JOINT(j);
    return deviceMapper.singleJointMapping(j, mode, &yarp::dev::IControlModeRaw::getControlModeRaw);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::getControlModes(int * modes)
{
    //CD_DEBUG("\n"); //-- Too verbose in controlboardwrapper2 stream
    return deviceMapper.fullJointMapping(modes, &yarp::dev::IControlModeRaw::getControlModesRaw);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::getControlModes(int n_joint, const int * joints, int * modes)
{
    CD_DEBUG("\n");
    return deviceMapper.multiJointMapping(n_joint, joints, modes, &yarp::dev::IControlModeRaw::getControlModesRaw);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::setControlMode(int j, int mode)
{
    CD_DEBUG("(%d, %s)\n", j, yarp::os::Vocab::decode(mode).c_str());
    CHECK_JOINT(j);

    if (!deviceMapper.singleJointMapping(j, mode, &yarp::dev::IControlModeRaw::setControlModeRaw))
    {
        return false;
    }

    posdThread->updateControlModeRegister(j, mode == VOCAB_CM_POSITION_DIRECT);
    return true;
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::setControlModes(int * modes)
{
    CD_DEBUG("\n");

    if (!deviceMapper.fullJointMapping(modes, &yarp::dev::IControlModeRaw::setControlModesRaw))
    {
        return false;
    }

    for (unsigned int i = 0; i < nodes.size(); i++)
    {
        posdThread->updateControlModeRegister(i, modes[i] == VOCAB_CM_POSITION_DIRECT);
    }

    return true;
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::setControlModes(int n_joint, const int * joints, int * modes)
{
    CD_DEBUG("(%d)\n",n_joint);

    if (!deviceMapper.multiJointMapping(n_joint, joints, modes, &yarp::dev::IControlModeRaw::setControlModesRaw))

    for (int i = 0; i < n_joint; i++)
    {
        posdThread->updateControlModeRegister(joints[i], modes[i] == VOCAB_CM_POSITION_DIRECT);
    }

    return true;
}

// -----------------------------------------------------------------------------
