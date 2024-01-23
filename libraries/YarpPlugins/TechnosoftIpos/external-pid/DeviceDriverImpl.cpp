// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "external-pid/TechnosoftIposExternal.hpp"

#include <yarp/os/LogStream.h>

#include "LogComponent.hpp"

using namespace roboticslab;

constexpr auto DEFAULT_ENABLE_CSV = false;

// -----------------------------------------------------------------------------

bool TechnosoftIposExternal::open(yarp::os::Searchable & config)
{
    yarp::os::Property options;
    options.fromString(config.findGroup("common").toString());
    options.fromString(config.toString(), false); // override common options

    auto enableCsvVal = options.check("enableCsv", yarp::os::Value(DEFAULT_ENABLE_CSV), "enable CSV mode");

    if (!setRemoteVariableRaw("enableCsv", {enableCsvVal}))
    {
        return false;
    }

    initialInteractionMode = options.check("initialInteractionMode", yarp::os::Value(yarp::dev::InteractionModeEnum::VOCAB_IM_UNKNOWN),
        "initial YARP interaction mode vocab").asVocab32();

    auto stiffness = options.check("stiffness", yarp::os::Value(0.0), "impedance stiffness (Nm)").asFloat64();
    auto damping = options.check("damping", yarp::os::Value(0.0), "impedance damping (Nm*seconds)").asFloat64();
    auto impedanceOffset = options.check("impedanceOffset", yarp::os::Value(0.0), "impedance offset").asFloat64();

    minStiffness = options.check("minStiffness", yarp::os::Value(0.0), "minimum impedance stiffness (Nm)").asFloat64();
    maxStiffness = options.check("maxStiffness", yarp::os::Value(0.0), "maximum impedance stiffness (Nm)").asFloat64();
    minDamping = options.check("minDamping", yarp::os::Value(0.0), "minimum impedance damping (Nm*seconds)").asFloat64();
    maxDamping = options.check("maxDamping", yarp::os::Value(0.0), "maximum impedance damping (Nm*seconds)").asFloat64();

    if (stiffness < minStiffness || stiffness > maxStiffness)
    {
        yCIError(IPOS, id(), "Invalid stiffness: %f (not in [%f, %f])", stiffness, minStiffness, maxStiffness);
        return false;
    }

    if (damping < minDamping || damping > maxDamping)
    {
        yCIError(IPOS, id(), "Invalid damping: %f (not in [%f, %f])", damping, minDamping, maxDamping);
        return false;
    }

    if (impedanceOffset < 0.0)
    {
        yCIError(IPOS, id()) << "Illegal offset:" << impedanceOffset;
        return false;
    }

    auto kp = options.check("kp", yarp::os::Value(0.0), "position PID Kp").asFloat64();
    auto ki = options.check("ki", yarp::os::Value(0.0), "position PID Ki").asFloat64();
    auto kd = options.check("kd", yarp::os::Value(0.0), "position PID Kd").asFloat64();
    auto maxInt = options.check("maxInt", yarp::os::Value(0.0), "position PID saturation threshold").asFloat64();
    auto maxOutput = options.check("maxOutput", yarp::os::Value(0.0), "position PID maximum output").asFloat64();
    auto offset = options.check("offset", yarp::os::Value(0.0), "position PID offset").asFloat64();
    auto scale = options.check("scale", yarp::os::Value(1.0), "position PID scale").asFloat64();
    auto stictionUp = options.check("stictionUp", yarp::os::Value(0.0), "position PID stiction up").asFloat64();
    auto stictionDown = options.check("stictionDown", yarp::os::Value(0.0), "position PID stiction down").asFloat64();
    auto kff = options.check("kff", yarp::os::Value(0.0), "position PID feed-forward").asFloat64();

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

    errorLimit = options.check("errorLimit", yarp::os::Value(0.0), "position PID error limit (degrees)").asFloat64();

    if (errorLimit <= 0.0)
    {
        yCIError(IPOS, id()) << "Illegal position error limit:" << errorLimit;
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
