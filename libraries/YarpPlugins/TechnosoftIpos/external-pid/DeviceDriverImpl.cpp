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

    initialInteractionMode = iposGroup.check("initialInteractionMode", yarp::os::Value(yarp::dev::InteractionModeEnum::VOCAB_IM_UNKNOWN),
        "initial YARP interaction mode vocab").asVocab32();

    auto stiffness = iposGroup.check("stiffness", yarp::os::Value(0.0), "impedance stiffness (Nm)").asFloat64();
    auto damping = iposGroup.check("damping", yarp::os::Value(0.0), "impedance damping (Nm*seconds)").asFloat64();
    auto impedanceOffset = iposGroup.check("impedanceOffset", yarp::os::Value(0.0), "impedance offset").asFloat64();

    minStiffness = iposGroup.check("minStiffness", yarp::os::Value(0.0), "minimum impedance stiffness (Nm)").asFloat64();
    maxStiffness = iposGroup.check("maxStiffness", yarp::os::Value(0.0), "maximum impedance stiffness (Nm)").asFloat64();
    minDamping = iposGroup.check("minDamping", yarp::os::Value(0.0), "minimum impedance damping (Nm*seconds)").asFloat64();
    maxDamping = iposGroup.check("maxDamping", yarp::os::Value(0.0), "maximum impedance damping (Nm*seconds)").asFloat64();

    if (stiffness < minStiffness || stiffness > maxStiffness)
    {
        yCError(IPOS, "Invalid stiffness: %f (not in [%f, %f])", stiffness, minStiffness, maxStiffness);
        return false;
    }

    if (damping < minDamping || damping > maxDamping)
    {
        yCError(IPOS, "Invalid damping: %f (not in [%f, %f])", damping, minDamping, maxDamping);
        return false;
    }

    if (impedanceOffset < 0.0)
    {
        yCError(IPOS) << "Illegal offset:" << impedanceOffset;
        return false;
    }

    auto kp = iposGroup.check("kp", yarp::os::Value(0.0), "position PID Kp").asFloat64();
    auto ki = iposGroup.check("ki", yarp::os::Value(0.0), "position PID Ki").asFloat64();
    auto kd = iposGroup.check("kd", yarp::os::Value(0.0), "position PID Kd").asFloat64();
    auto maxInt = iposGroup.check("maxInt", yarp::os::Value(0.0), "position PID saturation threshold").asFloat64();
    auto maxOutput = iposGroup.check("maxOutput", yarp::os::Value(0.0), "position PID maximum output").asFloat64();
    auto offset = iposGroup.check("offset", yarp::os::Value(0.0), "position PID offset").asFloat64();
    auto scale = iposGroup.check("scale", yarp::os::Value(1.0), "position PID scale").asFloat64();
    auto stictionUp = iposGroup.check("stictionUp", yarp::os::Value(0.0), "position PID stiction up").asFloat64();
    auto stictionDown = iposGroup.check("stictionDown", yarp::os::Value(0.0), "position PID stiction down").asFloat64();
    auto kff = iposGroup.check("kff", yarp::os::Value(0.0), "position PID feed-forward").asFloat64();

    positionPid.setKp(kp);
    positionPid.setKi(ki);
    positionPid.setKd(kd);
    positionPid.setMaxInt(maxInt);
    positionPid.setMaxOut(maxOutput);
    positionPid.setOffset(offset);
    positionPid.setScale(scale);
    positionPid.setStictionValues(stictionUp, stictionDown);
    positionPid.setKff(kff);

    impedancePid.setKp(stiffness);
    impedancePid.setKd(damping);
    impedancePid.setMaxOut(maxOutput); // re-use
    impedancePid.setOffset(impedanceOffset);
    impedancePid.setScale(1.0);
    impedancePid.setStictionValues(stictionUp, stictionDown); // re-use

    errorLimit = iposGroup.check("errorLimit", yarp::os::Value(0.0), "position PID error limit (degrees)").asFloat64();

    if (errorLimit <= 0.0)
    {
        yCError(IPOS) << "Illegal position error limit:" << errorLimit;
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
