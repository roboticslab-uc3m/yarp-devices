// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusControlboard.hpp"

#include <yarp/os/Vocab.h>

#include <ColorDebug.h>

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool CanBusControlboard::setPid(const yarp::dev::PidControlTypeEnum & pidtype, int j, const yarp::dev::Pid & pid)
{
    CD_DEBUG("(%s, %d)\n", yarp::os::Vocab::decode(pidtype).c_str(), j);
    CHECK_JOINT(j);

    int localAxis;
    yarp::dev::IPidControlRaw * p = deviceMapper.getDevice(j, &localAxis).iPidControlRaw;
    return p ? p->setPidRaw(pidtype, localAxis, pid) : false;
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::setPids(const yarp::dev::PidControlTypeEnum & pidtype, const yarp::dev::Pid * pids)
{
    CD_DEBUG("(%s)\n", yarp::os::Vocab::decode(pidtype).c_str());

    const int * localAxisOffsets;
    const std::vector<RawDevice> & rawDevices = deviceMapper.getDevices(localAxisOffsets);

    bool ok = true;

    for (int i = 0; i < rawDevices.size(); i++)
    {
        yarp::dev::IPidControlRaw * p = rawDevices[i].iPidControlRaw;
        ok &= p ? p->setPidsRaw(pidtype, pids + localAxisOffsets[i]) : false;
    }

    return ok;
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::setPidReference(const yarp::dev::PidControlTypeEnum & pidtype, int j, double ref)
{
    CD_DEBUG("(%s, %d, %f)\n", yarp::os::Vocab::decode(pidtype).c_str(), j, ref);
    CHECK_JOINT(j);

    int localAxis;
    yarp::dev::IPidControlRaw * p = deviceMapper.getDevice(j, &localAxis).iPidControlRaw;
    return p ? p->setPidReferenceRaw(pidtype, localAxis, ref) : false;
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::setPidReferences(const yarp::dev::PidControlTypeEnum & pidtype, const double * refs)
{
    CD_DEBUG("(%s)\n", yarp::os::Vocab::decode(pidtype).c_str());

    const int * localAxisOffsets;
    const std::vector<RawDevice> & rawDevices = deviceMapper.getDevices(localAxisOffsets);

    bool ok = true;

    for (int i = 0; i < rawDevices.size(); i++)
    {
        yarp::dev::IPidControlRaw * p = rawDevices[i].iPidControlRaw;
        ok &= p ? p->setPidReferencesRaw(pidtype, refs + localAxisOffsets[i]) : false;
    }

    return ok;
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::setPidErrorLimit(const yarp::dev::PidControlTypeEnum & pidtype, int j, double limit)
{
    CD_DEBUG("(%s, %d, %f)\n", yarp::os::Vocab::decode(pidtype).c_str(), j, limit);
    CHECK_JOINT(j);

    int localAxis;
    yarp::dev::IPidControlRaw * p = deviceMapper.getDevice(j, &localAxis).iPidControlRaw;
    return p ? p->setPidErrorLimitRaw(pidtype, localAxis, limit) : false;
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::setPidErrorLimits(const yarp::dev::PidControlTypeEnum & pidtype, const double * limits)
{
    CD_DEBUG("(%s)\n", yarp::os::Vocab::decode(pidtype).c_str());

    const int * localAxisOffsets;
    const std::vector<RawDevice> & rawDevices = deviceMapper.getDevices(localAxisOffsets);

    bool ok = true;

    for (int i = 0; i < rawDevices.size(); i++)
    {
        yarp::dev::IPidControlRaw * p = rawDevices[i].iPidControlRaw;
        ok &= p ? p->setPidErrorLimitsRaw(pidtype, limits + localAxisOffsets[i]) : false;
    }

    return ok;
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::getPidError(const yarp::dev::PidControlTypeEnum & pidtype, int j, double * err)
{
    CD_DEBUG("(%s, %d)\n", yarp::os::Vocab::decode(pidtype).c_str(), j);
    CHECK_JOINT(j);

    int localAxis;
    yarp::dev::IPidControlRaw * p = deviceMapper.getDevice(j, &localAxis).iPidControlRaw;
    return p ? p->getPidErrorRaw(pidtype, localAxis, err) : false;
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::getPidErrors(const yarp::dev::PidControlTypeEnum & pidtype, double * errs)
{
    CD_DEBUG("(%s)\n", yarp::os::Vocab::decode(pidtype).c_str());

    const int * localAxisOffsets;
    const std::vector<RawDevice> & rawDevices = deviceMapper.getDevices(localAxisOffsets);

    bool ok = true;

    for (int i = 0; i < rawDevices.size(); i++)
    {
        yarp::dev::IPidControlRaw * p = rawDevices[i].iPidControlRaw;
        ok &= p ? p->getPidErrorsRaw(pidtype, errs + localAxisOffsets[i]) : false;
    }

    return ok;
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::getPidOutput(const yarp::dev::PidControlTypeEnum & pidtype, int j, double *out)
{
    CD_DEBUG("(%s, %d)\n", yarp::os::Vocab::decode(pidtype).c_str(), j);
    CHECK_JOINT(j);

    int localAxis;
    yarp::dev::IPidControlRaw * p = deviceMapper.getDevice(j, &localAxis).iPidControlRaw;
    return p ? p->getPidOutputRaw(pidtype, localAxis, out) : false;
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::getPidOutputs(const yarp::dev::PidControlTypeEnum & pidtype, double * outs)
{
    CD_DEBUG("(%s)\n", yarp::os::Vocab::decode(pidtype).c_str());

    const int * localAxisOffsets;
    const std::vector<RawDevice> & rawDevices = deviceMapper.getDevices(localAxisOffsets);

    bool ok = true;

    for (int i = 0; i < rawDevices.size(); i++)
    {
        yarp::dev::IPidControlRaw * p = rawDevices[i].iPidControlRaw;
        ok &= p ? p->getPidOutputsRaw(pidtype, outs + localAxisOffsets[i]) : false;
    }

    return ok;
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::getPid(const yarp::dev::PidControlTypeEnum & pidtype, int j, yarp::dev::Pid * pid)
{
    CD_DEBUG("(%s, %d)\n", yarp::os::Vocab::decode(pidtype).c_str(), j);
    CHECK_JOINT(j);

    int localAxis;
    yarp::dev::IPidControlRaw * p = deviceMapper.getDevice(j, &localAxis).iPidControlRaw;
    return p ? p->getPidRaw(pidtype, localAxis, pid) : false;
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::getPids(const yarp::dev::PidControlTypeEnum & pidtype, yarp::dev::Pid * pids)
{
    CD_DEBUG("(%s)\n", yarp::os::Vocab::decode(pidtype).c_str());

    const int * localAxisOffsets;
    const std::vector<RawDevice> & rawDevices = deviceMapper.getDevices(localAxisOffsets);

    bool ok = true;

    for (int i = 0; i < rawDevices.size(); i++)
    {
        yarp::dev::IPidControlRaw * p = rawDevices[i].iPidControlRaw;
        ok &= p ? p->getPidsRaw(pidtype, pids + localAxisOffsets[i]) : false;
    }

    return ok;
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::getPidReference(const yarp::dev::PidControlTypeEnum & pidtype, int j, double * ref)
{
    CD_DEBUG("(%s, %d)\n", yarp::os::Vocab::decode(pidtype).c_str(), j);
    CHECK_JOINT(j);

    int localAxis;
    yarp::dev::IPidControlRaw * p = deviceMapper.getDevice(j, &localAxis).iPidControlRaw;
    return p ? p->getPidReferenceRaw(pidtype, localAxis, ref) : false;
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::getPidReferences(const yarp::dev::PidControlTypeEnum & pidtype, double * refs)
{
    CD_DEBUG("(%s)\n", yarp::os::Vocab::decode(pidtype).c_str());

    const int * localAxisOffsets;
    const std::vector<RawDevice> & rawDevices = deviceMapper.getDevices(localAxisOffsets);

    bool ok = true;

    for (int i = 0; i < rawDevices.size(); i++)
    {
        yarp::dev::IPidControlRaw * p = rawDevices[i].iPidControlRaw;
        ok &= p ? p->getPidReferencesRaw(pidtype, refs + localAxisOffsets[i]) : false;
    }

    return ok;
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::getPidErrorLimit(const yarp::dev::PidControlTypeEnum & pidtype, int j, double * limit)
{
    CD_DEBUG("(%s, %d)\n", yarp::os::Vocab::decode(pidtype).c_str(), j);
    CHECK_JOINT(j);

    int localAxis;
    yarp::dev::IPidControlRaw * p = deviceMapper.getDevice(j, &localAxis).iPidControlRaw;
    return p ? p->getPidErrorLimitRaw(pidtype, localAxis, limit) : false;
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::getPidErrorLimits(const yarp::dev::PidControlTypeEnum & pidtype, double * limits)
{
    CD_DEBUG("(%s)\n", yarp::os::Vocab::decode(pidtype).c_str());

    const int * localAxisOffsets;
    const std::vector<RawDevice> & rawDevices = deviceMapper.getDevices(localAxisOffsets);

    bool ok = true;

    for (int i = 0; i < rawDevices.size(); i++)
    {
        yarp::dev::IPidControlRaw * p = rawDevices[i].iPidControlRaw;
        ok &= p ? p->getPidErrorLimitsRaw(pidtype, limits + localAxisOffsets[i]) : false;
    }

    return ok;
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::resetPid(const yarp::dev::PidControlTypeEnum & pidtype, int j)
{
    CD_DEBUG("(%s, %d)\n", yarp::os::Vocab::decode(pidtype).c_str(), j);
    CHECK_JOINT(j);

    int localAxis;
    yarp::dev::IPidControlRaw * p = deviceMapper.getDevice(j, &localAxis).iPidControlRaw;
    return p ? p->resetPidRaw(pidtype, localAxis) : false;
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::disablePid(const yarp::dev::PidControlTypeEnum & pidtype, int j)
{
    CD_DEBUG("(%s, %d)\n", yarp::os::Vocab::decode(pidtype).c_str(), j);
    CHECK_JOINT(j);

    int localAxis;
    yarp::dev::IPidControlRaw * p = deviceMapper.getDevice(j, &localAxis).iPidControlRaw;
    return p ? p->disablePidRaw(pidtype, localAxis) : false;
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::enablePid(const yarp::dev::PidControlTypeEnum & pidtype, int j)
{
    CD_DEBUG("(%s, %d)\n", yarp::os::Vocab::decode(pidtype).c_str(), j);
    CHECK_JOINT(j);

    int localAxis;
    yarp::dev::IPidControlRaw * p = deviceMapper.getDevice(j, &localAxis).iPidControlRaw;
    return p ? p->enablePidRaw(pidtype, localAxis) : false;
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::setPidOffset(const yarp::dev::PidControlTypeEnum & pidtype, int j, double v)
{
    CD_DEBUG("(%s, %d, %f)\n", yarp::os::Vocab::decode(pidtype).c_str(), j, v);
    CHECK_JOINT(j);

    int localAxis;
    yarp::dev::IPidControlRaw * p = deviceMapper.getDevice(j, &localAxis).iPidControlRaw;
    return p ? p->setPidOffsetRaw(pidtype, localAxis, v) : false;
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::isPidEnabled(const yarp::dev::PidControlTypeEnum & pidtype, int j, bool * enabled)
{
    CD_DEBUG("(%s, %d)\n", yarp::os::Vocab::decode(pidtype).c_str(), j);
    CHECK_JOINT(j);

    int localAxis;
    yarp::dev::IPidControlRaw * p = deviceMapper.getDevice(j, &localAxis).iPidControlRaw;
    return p ? p->isPidEnabledRaw(pidtype, localAxis, enabled) : false;
}

// -----------------------------------------------------------------------------
