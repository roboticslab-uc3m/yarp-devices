// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "DextraRawControlboard.hpp"

#include <yarp/conf/version.h>
#include <yarp/os/Log.h>

#include "LogComponent.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool DextraRawControlboard::getAxisNameRaw(int axis, std::string & name)
{
#if defined(YARP_VERSION_COMPARE) // >= 3.6.0
    yCITrace(DEXTRA, id(), "%d", axis);
#else
    yCTrace(DEXTRA, "%d", axis);
#endif
    CHECK_JOINT(axis);
    name = Synapse::LABELS[axis];
    return true;
}

// -----------------------------------------------------------------------------

bool DextraRawControlboard::getJointTypeRaw(int axis, yarp::dev::JointTypeEnum & type)
{
#if defined(YARP_VERSION_COMPARE) // >= 3.6.0
    yCITrace(DEXTRA, id(), "%d", axis);
#else
    yCTrace(DEXTRA, "%d", axis);
#endif
    CHECK_JOINT(axis);
    type = yarp::dev::JointTypeEnum::VOCAB_JOINTTYPE_REVOLUTE;
    return true;
}

// -----------------------------------------------------------------------------
