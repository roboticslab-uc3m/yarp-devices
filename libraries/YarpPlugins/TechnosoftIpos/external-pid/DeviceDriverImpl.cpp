// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "external-pid/TechnosoftIposExternal.hpp"

#include <yarp/os/LogStream.h>

#include "LogComponent.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool TechnosoftIposExternal::open(yarp::os::Searchable & config)
{
    const auto * robotConfig = *reinterpret_cast<const yarp::os::Property * const *>(config.find("robotConfig").asBlob());

    const auto & commonGroup = robotConfig->findGroup("common-ipos");
    yarp::os::Property iposGroup;

    if (!commonGroup.isNull())
    {
        iposGroup.fromString(commonGroup.toString());
    }

    iposGroup.fromString(config.toString(), false); // override common options

    initialInteractionMode = iposGroup.check("initialInteractionMode", yarp::os::Value(yarp::dev::InteractionModeEnum::VOCAB_IM_STIFF),
        "initial YARP interaction mode vocab").asVocab32();

    impedanceStiffness = iposGroup.check("impedanceStiffness", yarp::os::Value(0.0), "impedance stiffness (Nm)").asFloat64();
    impedanceDamping = iposGroup.check("impedanceDamping", yarp::os::Value(0.0), "impedance damping (Nm*seconds)").asFloat64();
    impedanceOffset = iposGroup.check("impedanceOffset", yarp::os::Value(0.0), "impedance offset").asFloat64();

    minImpedanceStiffness = iposGroup.check("minImpedanceStiffness", yarp::os::Value(0.0), "minimum impedance stiffness (Nm)").asFloat64();
    maxImpedanceStiffness = iposGroup.check("maxImpedanceStiffness", yarp::os::Value(0.0), "maximum impedance stiffness (Nm)").asFloat64();
    minImpedanceDamping = iposGroup.check("minImpedanceDamping", yarp::os::Value(0.0), "minimum impedance damping (Nm*seconds)").asFloat64();
    maxImpedanceDamping = iposGroup.check("maxImpedanceDamping", yarp::os::Value(0.0), "maximum impedance damping (Nm*seconds)").asFloat64();

    if (impedanceStiffness < minImpedanceStiffness || impedanceStiffness > maxImpedanceStiffness)
    {
        yCIError(IPOS, id(), "Invalid stiffness: %f (not in [%f, %f])", impedanceStiffness.load(), minImpedanceStiffness, maxImpedanceStiffness);
        return false;
    }

    if (impedanceDamping < minImpedanceDamping || impedanceDamping > maxImpedanceDamping)
    {
        yCIError(IPOS, id(), "Invalid damping: %f (not in [%f, %f])", impedanceDamping.load(), minImpedanceDamping, maxImpedanceDamping);
        return false;
    }

    if (impedanceOffset < 0.0)
    {
        yCIError(IPOS, id()) << "Illegal offset:" << impedanceOffset.load();
        return false;
    }

    return TechnosoftIposBase::open(config);
}

// -----------------------------------------------------------------------------

bool TechnosoftIposExternal::close()
{
    return TechnosoftIposBase::close();
}

// -----------------------------------------------------------------------------
