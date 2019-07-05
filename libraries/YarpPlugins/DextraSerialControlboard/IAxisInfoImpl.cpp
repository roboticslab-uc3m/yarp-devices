// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "DextraSerialControlboard.hpp"

#include <ColorDebug.h>

// ------------------- IAxisInfo Related ------------------------------------

bool roboticslab::DextraSerialControlboard::getAxisName(int axis, std::string &name)
{
    CD_DEBUG("axis\n");
    CHECK_JOINT(axis);
    name = Synapse::LABELS[axis];
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::DextraSerialControlboard::getJointType(int axis, yarp::dev::JointTypeEnum &type)
{
    CD_DEBUG("axis\n");
    CHECK_JOINT(axis);
    type = yarp::dev::JointTypeEnum::VOCAB_JOINTTYPE_REVOLUTE;
    return true;
}

// -----------------------------------------------------------------------------
