// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "FakeJoint.hpp"

#include <yarp/os/LogStream.h>

using namespace roboticslab;

namespace
{
    YARP_LOG_COMPONENT(FJ, "rl.FakeJoint")
}

// -----------------------------------------------------------------------------

bool FakeJoint::open(yarp::os::Searchable & config)
{
    axes = config.check("axes", yarp::os::Value(1), "number of fake axes").asInt32();

    const auto * names = config.find("axisNames").asList();
    const auto * types = config.find("jointTypes").asList();

    if (names == nullptr)
    {
        yCIError(FJ, id()) << R"(Missing key "axisNames" or not a list)";
        return false;
    }

    if (types == nullptr)
    {
        yCIError(FJ, id()) << R"(Missing key "jointTypes" or not a list)";
        return false;
    }

    if (names->size() != axes || types->size() != axes)
    {
        yCIError(FJ, id()) << "Number of axes, names and types must match";
        return false;
    }

    for (int i = 0; i < axes; i++)
    {
        axisNames.push_back(names->get(i).asString());
        jointTypes.push_back(types->get(i).asVocab32());
    }

    controlModes.resize(axes, VOCAB_CM_CONFIGURED);
    interactionModes.resize(axes, yarp::dev::VOCAB_IM_UNKNOWN);

    return true;
}

// -----------------------------------------------------------------------------
