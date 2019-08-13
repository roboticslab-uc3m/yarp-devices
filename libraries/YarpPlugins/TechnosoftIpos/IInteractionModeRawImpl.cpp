// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TechnosoftIpos.hpp"

#include <yarp/os/Vocab.h>

// ------------------ IInteractionModeRaw Related ----------------------------------------


bool roboticslab::TechnosoftIpos::getInteractionModeRaw(int axis, yarp::dev::InteractionModeEnum* mode)
{
    CD_INFO("(%d)\n",axis);
    return true;
}

bool roboticslab::TechnosoftIpos::getInteractionModesRaw(int n_joints, int *joints, yarp::dev::InteractionModeEnum* modes)
{
    CD_WARNING("Missing implementation\n");
    return true;
}

bool roboticslab::TechnosoftIpos::getInteractionModesRaw(yarp::dev::InteractionModeEnum* modes)
{
    CD_WARNING("Missing implementation\n");
    return true;
}

bool roboticslab::TechnosoftIpos::setInteractionModeRaw(int axis, yarp::dev::InteractionModeEnum mode)
{
    CD_INFO("(%d), (%s)\n", axis, yarp::os::Vocab::decode(mode).c_str());
    return true;
}

bool roboticslab::TechnosoftIpos::setInteractionModesRaw(int n_joints, int *joints, yarp::dev::InteractionModeEnum* modes)
{
    CD_WARNING("Missing implementation\n");
    return true;
}

bool roboticslab::TechnosoftIpos::setInteractionModesRaw(yarp::dev::InteractionModeEnum* modes)
{
    CD_WARNING("Missing implementation\n");
    return true;
}
