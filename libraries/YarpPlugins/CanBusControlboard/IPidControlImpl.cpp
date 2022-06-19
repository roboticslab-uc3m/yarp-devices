// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusControlboard.hpp"

#include <yarp/os/Log.h>
#include <yarp/os/Vocab.h>

#include "LogComponent.hpp"

using namespace roboticslab;

namespace
{
    using pid_t = yarp::dev::PidControlTypeEnum;

    template<typename... T_ref>
    using single_mapping_fn = bool (yarp::dev::IPidControlRaw::*)(const pid_t &, int, T_ref...);

    template<typename... T_ref>
    bool mapSingleJoint(const DeviceMapper & dm, single_mapping_fn<T_ref...> fn, const pid_t & type, int j, T_ref... ref)
    {
        auto [device, offset] = dm.getDevice(j);
        auto * p = device->getHandle<yarp::dev::IPidControlRaw>();
        return p && (p->*fn)(type, offset, ref...);
    }

    template<typename T_refs>
    using full_mapping_fn = bool (yarp::dev::IPidControlRaw::*)(const pid_t &, T_refs *);

    template<typename T_refs>
    bool mapAllJoints(const DeviceMapper & dm, full_mapping_fn<T_refs> fn, const pid_t & type, T_refs * refs)
    {
        auto task = dm.createTask();
        bool ok = true;

        for (const auto & [device, offset] : dm.getDevicesWithOffsets())
        {
            auto * p = device->template getHandle<yarp::dev::IPidControlRaw>();
            ok &= p && (task->add(p, fn, type, refs + offset), true);
        }

        return ok && task->dispatch();
    }
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::setPid(const yarp::dev::PidControlTypeEnum & pidtype, int j, const yarp::dev::Pid & pid)
{
    yCTrace(CBCB, "%s %d", yarp::os::Vocab32::decode(pidtype).c_str(), j);
    CHECK_JOINT(j);
    return mapSingleJoint<const yarp::dev::Pid &>(deviceMapper, &yarp::dev::IPidControlRaw::setPidRaw, pidtype, j, pid);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::setPids(const yarp::dev::PidControlTypeEnum & pidtype, const yarp::dev::Pid * pids)
{
    yCTrace(CBCB, "%s", yarp::os::Vocab32::decode(pidtype).c_str());
    return mapAllJoints(deviceMapper, &yarp::dev::IPidControlRaw::setPidsRaw, pidtype, pids);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::setPidReference(const yarp::dev::PidControlTypeEnum & pidtype, int j, double ref)
{
    yCTrace(CBCB, "%s %d %f", yarp::os::Vocab32::decode(pidtype).c_str(), j, ref);
    CHECK_JOINT(j);
    return mapSingleJoint(deviceMapper, &yarp::dev::IPidControlRaw::setPidReferenceRaw, pidtype, j, ref);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::setPidReferences(const yarp::dev::PidControlTypeEnum & pidtype, const double * refs)
{
    yCTrace(CBCB, "%s", yarp::os::Vocab32::decode(pidtype).c_str());
    return mapAllJoints(deviceMapper, &yarp::dev::IPidControlRaw::setPidReferencesRaw, pidtype, refs);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::setPidErrorLimit(const yarp::dev::PidControlTypeEnum & pidtype, int j, double limit)
{
    yCTrace(CBCB, "%s %d %f", yarp::os::Vocab32::decode(pidtype).c_str(), j, limit);
    CHECK_JOINT(j);
    return mapSingleJoint(deviceMapper, &yarp::dev::IPidControlRaw::setPidErrorLimitRaw, pidtype, j, limit);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::setPidErrorLimits(const yarp::dev::PidControlTypeEnum & pidtype, const double * limits)
{
    yCTrace(CBCB, "%s", yarp::os::Vocab32::decode(pidtype).c_str());
    return mapAllJoints(deviceMapper, &yarp::dev::IPidControlRaw::setPidErrorLimitsRaw, pidtype, limits);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::getPidError(const yarp::dev::PidControlTypeEnum & pidtype, int j, double * err)
{
    yCTrace(CBCB, "%s %d", yarp::os::Vocab32::decode(pidtype).c_str(), j);
    CHECK_JOINT(j);
    return mapSingleJoint(deviceMapper, &yarp::dev::IPidControlRaw::getPidErrorRaw, pidtype, j, err);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::getPidErrors(const yarp::dev::PidControlTypeEnum & pidtype, double * errs)
{
    yCTrace(CBCB, "%s", yarp::os::Vocab32::decode(pidtype).c_str());
    return mapAllJoints(deviceMapper, &yarp::dev::IPidControlRaw::getPidErrorsRaw, pidtype, errs);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::getPidOutput(const yarp::dev::PidControlTypeEnum & pidtype, int j, double *out)
{
    yCTrace(CBCB, "%s %d", yarp::os::Vocab32::decode(pidtype).c_str(), j);
    CHECK_JOINT(j);
    return mapSingleJoint(deviceMapper, &yarp::dev::IPidControlRaw::getPidOutputRaw, pidtype, j, out);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::getPidOutputs(const yarp::dev::PidControlTypeEnum & pidtype, double * outs)
{
    yCTrace(CBCB, "%s", yarp::os::Vocab32::decode(pidtype).c_str());
    return mapAllJoints(deviceMapper, &yarp::dev::IPidControlRaw::getPidOutputsRaw, pidtype, outs);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::getPid(const yarp::dev::PidControlTypeEnum & pidtype, int j, yarp::dev::Pid * pid)
{
    yCTrace(CBCB, "%s %d", yarp::os::Vocab32::decode(pidtype).c_str(), j);
    CHECK_JOINT(j);
    return mapSingleJoint(deviceMapper, &yarp::dev::IPidControlRaw::getPidRaw, pidtype, j, pid);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::getPids(const yarp::dev::PidControlTypeEnum & pidtype, yarp::dev::Pid * pids)
{
    yCTrace(CBCB, "%s", yarp::os::Vocab32::decode(pidtype).c_str());
    return mapAllJoints(deviceMapper, &yarp::dev::IPidControlRaw::getPidsRaw, pidtype, pids);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::getPidReference(const yarp::dev::PidControlTypeEnum & pidtype, int j, double * ref)
{
    yCTrace(CBCB, "%s %d", yarp::os::Vocab32::decode(pidtype).c_str(), j);
    CHECK_JOINT(j);
    return mapSingleJoint(deviceMapper, &yarp::dev::IPidControlRaw::getPidReferenceRaw, pidtype, j, ref);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::getPidReferences(const yarp::dev::PidControlTypeEnum & pidtype, double * refs)
{
    yCTrace(CBCB, "%s", yarp::os::Vocab32::decode(pidtype).c_str());
    return mapAllJoints(deviceMapper, &yarp::dev::IPidControlRaw::getPidReferencesRaw, pidtype, refs);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::getPidErrorLimit(const yarp::dev::PidControlTypeEnum & pidtype, int j, double * limit)
{
    yCTrace(CBCB, "%s %d", yarp::os::Vocab32::decode(pidtype).c_str(), j);
    CHECK_JOINT(j);
    return mapSingleJoint(deviceMapper, &yarp::dev::IPidControlRaw::getPidErrorLimitRaw, pidtype, j, limit);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::getPidErrorLimits(const yarp::dev::PidControlTypeEnum & pidtype, double * limits)
{
    yCTrace(CBCB, "%s", yarp::os::Vocab32::decode(pidtype).c_str());
    return mapAllJoints(deviceMapper, &yarp::dev::IPidControlRaw::getPidErrorLimitsRaw, pidtype, limits);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::resetPid(const yarp::dev::PidControlTypeEnum & pidtype, int j)
{
    yCTrace(CBCB, "%s %d", yarp::os::Vocab32::decode(pidtype).c_str(), j);
    CHECK_JOINT(j);
    return mapSingleJoint(deviceMapper, &yarp::dev::IPidControlRaw::resetPidRaw, pidtype, j);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::disablePid(const yarp::dev::PidControlTypeEnum & pidtype, int j)
{
    yCTrace(CBCB, "%s %d", yarp::os::Vocab32::decode(pidtype).c_str(), j);
    CHECK_JOINT(j);
    return mapSingleJoint(deviceMapper, &yarp::dev::IPidControlRaw::disablePidRaw, pidtype, j);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::enablePid(const yarp::dev::PidControlTypeEnum & pidtype, int j)
{
    yCTrace(CBCB, "%s %d", yarp::os::Vocab32::decode(pidtype).c_str(), j);
    CHECK_JOINT(j);
    return mapSingleJoint(deviceMapper, &yarp::dev::IPidControlRaw::enablePidRaw, pidtype, j);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::setPidOffset(const yarp::dev::PidControlTypeEnum & pidtype, int j, double v)
{
    yCTrace(CBCB, "%s %d %f", yarp::os::Vocab32::decode(pidtype).c_str(), j, v);
    CHECK_JOINT(j);
    return mapSingleJoint(deviceMapper, &yarp::dev::IPidControlRaw::setPidOffsetRaw, pidtype, j, v);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::isPidEnabled(const yarp::dev::PidControlTypeEnum & pidtype, int j, bool * enabled)
{
    yCTrace(CBCB, "%s %d", yarp::os::Vocab32::decode(pidtype).c_str(), j);
    CHECK_JOINT(j);
    return mapSingleJoint(deviceMapper, &yarp::dev::IPidControlRaw::isPidEnabledRaw, pidtype, j, enabled);
}

// -----------------------------------------------------------------------------
