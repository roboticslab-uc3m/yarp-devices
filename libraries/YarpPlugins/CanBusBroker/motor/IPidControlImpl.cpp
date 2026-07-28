// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusBroker.hpp"

#include <functional> // std::invoke

using namespace roboticslab;

namespace
{
    using pid_t = yarp::dev::PidControlTypeEnum;

    template<typename... T_ref>
    using single_joint_fn = bool (yarp::dev::IPidControlRaw::*)(const pid_t &, int, T_ref...);

    template<typename... T_ref>
    bool mapSingleJoint(const DeviceMapper & dm, single_joint_fn<T_ref...> fn, const pid_t & type, int j, T_ref... ref)
    {
        auto [device, offset] = dm.getMotorDevice(j);
        auto * p = device->getHandle<yarp::dev::IPidControlRaw>();
        return p && std::invoke(fn, p, type, offset, ref...);
    }

    template<typename T_refs>
    using all_joints_fn = bool (yarp::dev::IPidControlRaw::*)(const pid_t &, T_refs *);

    template<typename T_refs>
    bool mapAllJoints(const DeviceMapper & dm, all_joints_fn<T_refs> fn, const pid_t & type, T_refs * refs)
    {
        auto task = dm.createTask();
        bool ok = true;

        for (const auto & [device, offset] : dm.getMotorDevicesWithOffsets())
        {
            auto * p = device->template getHandle<yarp::dev::IPidControlRaw>();
            ok &= p && (task->add(p, fn, type, refs + offset), true);
        }

        return ok && task->dispatch();
    }
}

// -----------------------------------------------------------------------------

bool CanBusBroker::setPid(const yarp::dev::PidControlTypeEnum & pidtype, int j, const yarp::dev::Pid & pid)
{
    CHECK_JOINT(j);
    return mapSingleJoint<const yarp::dev::Pid &>(deviceMapper, &yarp::dev::IPidControlRaw::setPidRaw, pidtype, j, pid);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::setPids(const yarp::dev::PidControlTypeEnum & pidtype, const yarp::dev::Pid * pids)
{
    return mapAllJoints(deviceMapper, &yarp::dev::IPidControlRaw::setPidsRaw, pidtype, pids);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::setPidReference(const yarp::dev::PidControlTypeEnum & pidtype, int j, double ref)
{
    CHECK_JOINT(j);
    return mapSingleJoint(deviceMapper, &yarp::dev::IPidControlRaw::setPidReferenceRaw, pidtype, j, ref);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::setPidReferences(const yarp::dev::PidControlTypeEnum & pidtype, const double * refs)
{
    return mapAllJoints(deviceMapper, &yarp::dev::IPidControlRaw::setPidReferencesRaw, pidtype, refs);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::setPidErrorLimit(const yarp::dev::PidControlTypeEnum & pidtype, int j, double limit)
{
    CHECK_JOINT(j);
    return mapSingleJoint(deviceMapper, &yarp::dev::IPidControlRaw::setPidErrorLimitRaw, pidtype, j, limit);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::setPidErrorLimits(const yarp::dev::PidControlTypeEnum & pidtype, const double * limits)
{
    return mapAllJoints(deviceMapper, &yarp::dev::IPidControlRaw::setPidErrorLimitsRaw, pidtype, limits);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::getPidError(const yarp::dev::PidControlTypeEnum & pidtype, int j, double * err)
{
    CHECK_JOINT(j);
    return mapSingleJoint(deviceMapper, &yarp::dev::IPidControlRaw::getPidErrorRaw, pidtype, j, err);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::getPidErrors(const yarp::dev::PidControlTypeEnum & pidtype, double * errs)
{
    return mapAllJoints(deviceMapper, &yarp::dev::IPidControlRaw::getPidErrorsRaw, pidtype, errs);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::getPidOutput(const yarp::dev::PidControlTypeEnum & pidtype, int j, double *out)
{
    CHECK_JOINT(j);
    return mapSingleJoint(deviceMapper, &yarp::dev::IPidControlRaw::getPidOutputRaw, pidtype, j, out);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::getPidOutputs(const yarp::dev::PidControlTypeEnum & pidtype, double * outs)
{
    return mapAllJoints(deviceMapper, &yarp::dev::IPidControlRaw::getPidOutputsRaw, pidtype, outs);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::getPid(const yarp::dev::PidControlTypeEnum & pidtype, int j, yarp::dev::Pid * pid)
{
    CHECK_JOINT(j);
    return mapSingleJoint(deviceMapper, &yarp::dev::IPidControlRaw::getPidRaw, pidtype, j, pid);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::getPids(const yarp::dev::PidControlTypeEnum & pidtype, yarp::dev::Pid * pids)
{
    return mapAllJoints(deviceMapper, &yarp::dev::IPidControlRaw::getPidsRaw, pidtype, pids);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::getPidReference(const yarp::dev::PidControlTypeEnum & pidtype, int j, double * ref)
{
    CHECK_JOINT(j);
    return mapSingleJoint(deviceMapper, &yarp::dev::IPidControlRaw::getPidReferenceRaw, pidtype, j, ref);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::getPidReferences(const yarp::dev::PidControlTypeEnum & pidtype, double * refs)
{
    return mapAllJoints(deviceMapper, &yarp::dev::IPidControlRaw::getPidReferencesRaw, pidtype, refs);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::getPidErrorLimit(const yarp::dev::PidControlTypeEnum & pidtype, int j, double * limit)
{
    CHECK_JOINT(j);
    return mapSingleJoint(deviceMapper, &yarp::dev::IPidControlRaw::getPidErrorLimitRaw, pidtype, j, limit);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::getPidErrorLimits(const yarp::dev::PidControlTypeEnum & pidtype, double * limits)
{
    return mapAllJoints(deviceMapper, &yarp::dev::IPidControlRaw::getPidErrorLimitsRaw, pidtype, limits);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::resetPid(const yarp::dev::PidControlTypeEnum & pidtype, int j)
{
    CHECK_JOINT(j);
    return mapSingleJoint(deviceMapper, &yarp::dev::IPidControlRaw::resetPidRaw, pidtype, j);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::disablePid(const yarp::dev::PidControlTypeEnum & pidtype, int j)
{
    CHECK_JOINT(j);
    return mapSingleJoint(deviceMapper, &yarp::dev::IPidControlRaw::disablePidRaw, pidtype, j);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::enablePid(const yarp::dev::PidControlTypeEnum & pidtype, int j)
{
    CHECK_JOINT(j);
    return mapSingleJoint(deviceMapper, &yarp::dev::IPidControlRaw::enablePidRaw, pidtype, j);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::setPidOffset(const yarp::dev::PidControlTypeEnum & pidtype, int j, double v)
{
    CHECK_JOINT(j);
    return mapSingleJoint(deviceMapper, &yarp::dev::IPidControlRaw::setPidOffsetRaw, pidtype, j, v);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::isPidEnabled(const yarp::dev::PidControlTypeEnum & pidtype, int j, bool * enabled)
{
    CHECK_JOINT(j);
    return mapSingleJoint(deviceMapper, &yarp::dev::IPidControlRaw::isPidEnabledRaw, pidtype, j, enabled);
}

// -----------------------------------------------------------------------------
