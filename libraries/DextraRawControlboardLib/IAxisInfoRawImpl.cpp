// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "DextraRawControlboard.hpp"

#include <yarp/os/Log.h>

#include "LogComponent.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool DextraRawControlboard::getAxisNameRaw(int axis, std::string & name)
{
    yCITrace(DEXTRA, id(), "%d", axis);
    CHECK_JOINT(axis);
    name = Synapse::LABELS[axis];
    return true;
}

// -----------------------------------------------------------------------------

bool DextraRawControlboard::getJointTypeRaw(int axis, yarp::dev::JointTypeEnum & type)
{
    yCITrace(DEXTRA, id(), "%d", axis);
    CHECK_JOINT(axis);
    type = yarp::dev::JointTypeEnum::VOCAB_JOINTTYPE_REVOLUTE;
    return true;
}

// -----------------------------------------------------------------------------
