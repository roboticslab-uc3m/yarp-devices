// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "DextraRawControlBoard.hpp"

#include <yarp/os/Log.h>

#include "LogComponent.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool DextraRawControlBoard::getAxisNameRaw(int axis, std::string & name)
{
    yCITrace(DEXTRA, id(), "%d", axis);
    CHECK_JOINT(axis);
    name = axisPrefix + Synapse::LABELS[axis];
    return true;
}

// -----------------------------------------------------------------------------

bool DextraRawControlBoard::getJointTypeRaw(int axis, yarp::dev::JointTypeEnum & type)
{
    yCITrace(DEXTRA, id(), "%d", axis);
    CHECK_JOINT(axis);
    type = yarp::dev::JointTypeEnum::VOCAB_JOINTTYPE_REVOLUTE;
    return true;
}

// -----------------------------------------------------------------------------
