// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusControlBoard.hpp"

#include <functional> // std::invoke

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
        return p && std::invoke(fn, p, type, offset, ref...);
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

bool CanBusControlBoard::setPid(const yarp::dev::PidControlTypeEnum & pidtype, int j, const yarp::dev::Pid & pid)
{
    yCTrace(CBCB, "%s %d", yarp::os::Vocab32::decode(pidtype).c_str(), j);
    CHECK_JOINT(j);
    return mapSingleJoint<const yarp::dev::Pid &>(deviceMapper, &yarp::dev::IPidControlRaw::setPidRaw, pidtype, j, pid);
}

// -----------------------------------------------------------------------------

bool CanBusControlBoard::setPids(const yarp::dev::PidControlTypeEnum & pidtype, const yarp::dev::Pid * pids)
{
    yCTrace(CBCB, "%s", yarp::os::Vocab32::decode(pidtype).c_str());
    return mapAllJoints(deviceMapper, &yarp::dev::IPidControlRaw::setPidsRaw, pidtype, pids);
}

// -----------------------------------------------------------------------------

bool CanBusControlBoard::setPidReference(const yarp::dev::PidControlTypeEnum & pidtype, int j, double ref)
{
    yCTrace(CBCB, "%s %d %f", yarp::os::Vocab32::decode(pidtype).c_str(), j, ref);
    CHECK_JOINT(j);
    return mapSingleJoint(deviceMapper, &yarp::dev::IPidControlRaw::setPidReferenceRaw, pidtype, j, ref);
}

// -----------------------------------------------------------------------------

bool CanBusControlBoard::setPidReferences(const yarp::dev::PidControlTypeEnum & pidtype, const double * refs)
{
    yCTrace(CBCB, "%s", yarp::os::Vocab32::decode(pidtype).c_str());
    return mapAllJoints(deviceMapper, &yarp::dev::IPidControlRaw::setPidReferencesRaw, pidtype, refs);
}

// -----------------------------------------------------------------------------

bool CanBusControlBoard::setPidErrorLimit(const yarp::dev::PidControlTypeEnum & pidtype, int j, double limit)
{
    yCTrace(CBCB, "%s %d %f", yarp::os::Vocab32::decode(pidtype).c_str(), j, limit);
    CHECK_JOINT(j);
    return mapSingleJoint(deviceMapper, &yarp::dev::IPidControlRaw::setPidErrorLimitRaw, pidtype, j, limit);
}

// -----------------------------------------------------------------------------

bool CanBusControlBoard::setPidErrorLimits(const yarp::dev::PidControlTypeEnum & pidtype, const double * limits)
{
    yCTrace(CBCB, "%s", yarp::os::Vocab32::decode(pidtype).c_str());
    return mapAllJoints(deviceMapper, &yarp::dev::IPidControlRaw::setPidErrorLimitsRaw, pidtype, limits);
}

// -----------------------------------------------------------------------------

bool CanBusControlBoard::getPidError(const yarp::dev::PidControlTypeEnum & pidtype, int j, double * err)
{
    yCTrace(CBCB, "%s %d", yarp::os::Vocab32::decode(pidtype).c_str(), j);
    CHECK_JOINT(j);
    return mapSingleJoint(deviceMapper, &yarp::dev::IPidControlRaw::getPidErrorRaw, pidtype, j, err);
}

// -----------------------------------------------------------------------------

bool CanBusControlBoard::getPidErrors(const yarp::dev::PidControlTypeEnum & pidtype, double * errs)
{
    yCTrace(CBCB, "%s", yarp::os::Vocab32::decode(pidtype).c_str());
    return mapAllJoints(deviceMapper, &yarp::dev::IPidControlRaw::getPidErrorsRaw, pidtype, errs);
}

// -----------------------------------------------------------------------------

bool CanBusControlBoard::getPidOutput(const yarp::dev::PidControlTypeEnum & pidtype, int j, double *out)
{
    yCTrace(CBCB, "%s %d", yarp::os::Vocab32::decode(pidtype).c_str(), j);
    CHECK_JOINT(j);
    return mapSingleJoint(deviceMapper, &yarp::dev::IPidControlRaw::getPidOutputRaw, pidtype, j, out);
}

// -----------------------------------------------------------------------------

bool CanBusControlBoard::getPidOutputs(const yarp::dev::PidControlTypeEnum & pidtype, double * outs)
{
    yCTrace(CBCB, "%s", yarp::os::Vocab32::decode(pidtype).c_str());
    return mapAllJoints(deviceMapper, &yarp::dev::IPidControlRaw::getPidOutputsRaw, pidtype, outs);
}

// -----------------------------------------------------------------------------

bool CanBusControlBoard::getPid(const yarp::dev::PidControlTypeEnum & pidtype, int j, yarp::dev::Pid * pid)
{
    yCTrace(CBCB, "%s %d", yarp::os::Vocab32::decode(pidtype).c_str(), j);
    CHECK_JOINT(j);
    return mapSingleJoint(deviceMapper, &yarp::dev::IPidControlRaw::getPidRaw, pidtype, j, pid);
}

// -----------------------------------------------------------------------------

bool CanBusControlBoard::getPids(const yarp::dev::PidControlTypeEnum & pidtype, yarp::dev::Pid * pids)
{
    yCTrace(CBCB, "%s", yarp::os::Vocab32::decode(pidtype).c_str());
    return mapAllJoints(deviceMapper, &yarp::dev::IPidControlRaw::getPidsRaw, pidtype, pids);
}

// -----------------------------------------------------------------------------

bool CanBusControlBoard::getPidReference(const yarp::dev::PidControlTypeEnum & pidtype, int j, double * ref)
{
    yCTrace(CBCB, "%s %d", yarp::os::Vocab32::decode(pidtype).c_str(), j);
    CHECK_JOINT(j);
    return mapSingleJoint(deviceMapper, &yarp::dev::IPidControlRaw::getPidReferenceRaw, pidtype, j, ref);
}

// -----------------------------------------------------------------------------

bool CanBusControlBoard::getPidReferences(const yarp::dev::PidControlTypeEnum & pidtype, double * refs)
{
    yCTrace(CBCB, "%s", yarp::os::Vocab32::decode(pidtype).c_str());
    return mapAllJoints(deviceMapper, &yarp::dev::IPidControlRaw::getPidReferencesRaw, pidtype, refs);
}

// -----------------------------------------------------------------------------

bool CanBusControlBoard::getPidErrorLimit(const yarp::dev::PidControlTypeEnum & pidtype, int j, double * limit)
{
    yCTrace(CBCB, "%s %d", yarp::os::Vocab32::decode(pidtype).c_str(), j);
    CHECK_JOINT(j);
    return mapSingleJoint(deviceMapper, &yarp::dev::IPidControlRaw::getPidErrorLimitRaw, pidtype, j, limit);
}

// -----------------------------------------------------------------------------

bool CanBusControlBoard::getPidErrorLimits(const yarp::dev::PidControlTypeEnum & pidtype, double * limits)
{
    yCTrace(CBCB, "%s", yarp::os::Vocab32::decode(pidtype).c_str());
    return mapAllJoints(deviceMapper, &yarp::dev::IPidControlRaw::getPidErrorLimitsRaw, pidtype, limits);
}

// -----------------------------------------------------------------------------

bool CanBusControlBoard::resetPid(const yarp::dev::PidControlTypeEnum & pidtype, int j)
{
    yCTrace(CBCB, "%s %d", yarp::os::Vocab32::decode(pidtype).c_str(), j);
    CHECK_JOINT(j);
    return mapSingleJoint(deviceMapper, &yarp::dev::IPidControlRaw::resetPidRaw, pidtype, j);
}

// -----------------------------------------------------------------------------

bool CanBusControlBoard::disablePid(const yarp::dev::PidControlTypeEnum & pidtype, int j)
{
    yCTrace(CBCB, "%s %d", yarp::os::Vocab32::decode(pidtype).c_str(), j);
    CHECK_JOINT(j);
    return mapSingleJoint(deviceMapper, &yarp::dev::IPidControlRaw::disablePidRaw, pidtype, j);
}

// -----------------------------------------------------------------------------

bool CanBusControlBoard::enablePid(const yarp::dev::PidControlTypeEnum & pidtype, int j)
{
    yCTrace(CBCB, "%s %d", yarp::os::Vocab32::decode(pidtype).c_str(), j);
    CHECK_JOINT(j);
    return mapSingleJoint(deviceMapper, &yarp::dev::IPidControlRaw::enablePidRaw, pidtype, j);
}

// -----------------------------------------------------------------------------

bool CanBusControlBoard::setPidOffset(const yarp::dev::PidControlTypeEnum & pidtype, int j, double v)
{
    yCTrace(CBCB, "%s %d %f", yarp::os::Vocab32::decode(pidtype).c_str(), j, v);
    CHECK_JOINT(j);
    return mapSingleJoint(deviceMapper, &yarp::dev::IPidControlRaw::setPidOffsetRaw, pidtype, j, v);
}

// -----------------------------------------------------------------------------

bool CanBusControlBoard::isPidEnabled(const yarp::dev::PidControlTypeEnum & pidtype, int j, bool * enabled)
{
    yCTrace(CBCB, "%s %d", yarp::os::Vocab32::decode(pidtype).c_str(), j);
    CHECK_JOINT(j);
    return mapSingleJoint(deviceMapper, &yarp::dev::IPidControlRaw::isPidEnabledRaw, pidtype, j, enabled);
}

// -----------------------------------------------------------------------------
