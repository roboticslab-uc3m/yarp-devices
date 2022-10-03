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

    if (!config.check("syncPeriod"))
    {
        yCIError(IPOS, id()) << "Missing mandatory --syncPeriod parameter";
        return false;
    }

    auto initialInteractionMode = iposGroup.check("initialInteractionMode", yarp::os::Value(yarp::dev::InteractionModeEnum::VOCAB_IM_UNKNOWN),
        "initial YARP interaction mode vocab").asVocab32();

    if (!setInteractionModeRaw(0, static_cast<yarp::dev::InteractionModeEnum>(initialInteractionMode)))
    {
        yCIError(IPOS, id()) << "Unable to configure initial interaction mode";
        return false;
    }

    auto impedanceStiffness = iposGroup.check("impedanceStiffness", yarp::os::Value(0.0), "impedance stiffness (Nm)").asFloat64();
    auto impedanceDamping = iposGroup.check("impedanceDamping", yarp::os::Value(0.0), "impedance damping (Nm*seconds)").asFloat64();
    auto impedanceOffset = iposGroup.check("impedanceOffset", yarp::os::Value(0.0), "impedance offset").asFloat64();

    minImpedanceStiffness = iposGroup.check("minImpedanceStiffness", yarp::os::Value(0.0), "minimum impedance stiffness (Nm)").asFloat64();
    maxImpedanceStiffness = iposGroup.check("maxImpedanceStiffness", yarp::os::Value(0.0), "maximum impedance stiffness (Nm)").asFloat64();
    minImpedanceDamping = iposGroup.check("minImpedanceDamping", yarp::os::Value(0.0), "minimum impedance damping (Nm*seconds)").asFloat64();
    maxImpedanceDamping = iposGroup.check("maxImpedanceDamping", yarp::os::Value(0.0), "maximum impedance damping (Nm*seconds)").asFloat64();

    if (impedanceStiffness < minImpedanceStiffness || impedanceStiffness > maxImpedanceStiffness)
    {
        yCIError(IPOS, id(), "Invalid stiffness: %f (not in [%f, %f])", impedanceStiffness, minImpedanceStiffness, maxImpedanceStiffness);
        return false;
    }

    if (impedanceDamping < minImpedanceDamping || impedanceDamping > maxImpedanceDamping)
    {
        yCIError(IPOS, id(), "Invalid damping: %f (not in [%f, %f])", impedanceDamping, minImpedanceDamping, maxImpedanceDamping);
        return false;
    }

    if (impedanceOffset < 0.0)
    {
        yCIError(IPOS, id()) << "Illegal offset:" << impedanceOffset;
        return false;
    }

    auto positionKp = iposGroup.check("positionKp", yarp::os::Value(0.0), "position PID Kp").asFloat64();
    auto positionKi = iposGroup.check("positionKi", yarp::os::Value(0.0), "position PID Ki").asFloat64();
    auto positionKd = iposGroup.check("positionKd", yarp::os::Value(0.0), "position PID Kd").asFloat64();
    auto positionMaxInt = iposGroup.check("positionMaxInt", yarp::os::Value(0.0), "position PID saturation threshold").asFloat64();
    auto positionMaxOutput = iposGroup.check("positionMaxOutput", yarp::os::Value(0.0), "position PID maximum output").asFloat64();
    auto positionOffset = iposGroup.check("positionOffset", yarp::os::Value(0.0), "position PID offset").asFloat64();
    auto positionScale = iposGroup.check("positionScale", yarp::os::Value(1.0), "position PID scale").asFloat64();
    auto positionStictionUp = iposGroup.check("positionStictionUp", yarp::os::Value(0.0), "position PID stiction up").asFloat64();
    auto positionStictionDown = iposGroup.check("positionStictionDown", yarp::os::Value(0.0), "position PID stiction down").asFloat64();
    auto positionKff = iposGroup.check("positionKff", yarp::os::Value(0.0), "position PID feed-forward").asFloat64();

    positionPid.setKp(positionKp);
    positionPid.setKi(positionKi);
    positionPid.setKd(positionKd);
    positionPid.setMaxInt(positionMaxInt);
    positionPid.setMaxOut(positionMaxOutput);
    positionPid.setOffset(positionOffset);
    positionPid.setScale(positionScale);
    positionPid.setStictionValues(positionStictionUp, positionStictionDown);
    positionPid.setKff(positionKff);

    impedancePid.setKp(impedanceStiffness);
    impedancePid.setKd(impedanceDamping);
    impedancePid.setMaxOut(positionMaxOutput); // re-use
    impedancePid.setOffset(impedanceOffset);
    impedancePid.setScale(1.0);
    impedancePid.setStictionValues(positionStictionUp, positionStictionDown); // re-use

    positionErrorLimit = iposGroup.check("positionErrorLimit", yarp::os::Value(0.0), "position PID error limit (degrees)").asFloat64();

    if (positionErrorLimit < 0.0)
    {
        yCIError(IPOS, id()) << "Illegal position error limit:" << positionErrorLimit;
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
