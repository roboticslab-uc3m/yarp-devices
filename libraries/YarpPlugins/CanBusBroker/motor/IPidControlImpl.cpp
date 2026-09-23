// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusBroker.hpp"

#include <functional> // std::invoke

using namespace roboticslab;

namespace
{
    using pid_t = yarp::dev::PidControlTypeEnum;
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    using return_t = yarp::dev::ReturnValue;
#else
    using return_t = bool;
#endif

    template<typename... T_ref>
    using single_joint_fn = return_t (yarp::dev::IPidControlRaw::*)(const pid_t &, int, T_ref...);

    template<typename... T_ref, typename... Args>
    return_t mapSingleJoint(const DeviceMapper & dm, single_joint_fn<T_ref...> fn, const pid_t & type, int j, Args &&... args)
    {
        auto [device, offset] = dm.getMotorDevice(j);
        auto * p = device->getHandle<yarp::dev::IPidControlRaw>();
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
        return p ? std::invoke(fn, p, type, offset, args...) : yarp::dev::ReturnValue_error_method_failed;
#else
        return p && std::invoke(fn, p, type, offset, args...);
#endif
    }

    template<typename T_refs>
    using all_joints_fn = return_t (yarp::dev::IPidControlRaw::*)(const pid_t &, T_refs *);

    template<typename T_refs>
    return_t mapAllJoints(const DeviceMapper & dm, all_joints_fn<T_refs> fn, const pid_t & type, T_refs * refs)
    {
        auto task = dm.createTask();
        bool ok = true;

        for (const auto & [device, offset] : dm.getMotorDevicesWithOffsets())
        {
            auto * p = device->template getHandle<yarp::dev::IPidControlRaw>();
            ok &= p && (task->add(p, fn, type, refs + offset), true);
        }

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
        return ok && task->dispatch()
            ? yarp::dev::ReturnValue_ok
            : yarp::dev::ReturnValue_error_method_failed;
#else
        return ok && task->dispatch();
#endif
    }

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    template<typename Elem>
    using all_joints_vec_fn = return_t (yarp::dev::IPidControlRaw::*)(const pid_t &, std::vector<Elem> &);

    template<typename Elem>
    return_t mapAllJoints(const DeviceMapper & dm, all_joints_vec_fn<Elem> fn, const pid_t & type, std::vector<Elem> & vec)
    {
        auto task = dm.createTask();
        bool ok = true;
        const auto & devices = dm.getMotorDevicesWithOffsets();

        struct DeviceSubSlice
        {
            std::size_t offset;
            std::vector<Elem> sub_vec;
        };

        auto slices = std::make_shared<std::vector<DeviceSubSlice>>();

        for (size_t i = 0; i < devices.size(); ++i)
        {
            auto [device, offset] = devices[i];
            auto * p = device->template getHandle<yarp::dev::IPidControlRaw>();

            if (p)
            {
                auto u_offset = static_cast<size_t>(offset);
                auto next_offset = (i + 1 < devices.size()) ? static_cast<size_t>(std::get<1>(devices[i + 1])) : vec.size();

                slices->push_back({u_offset, std::vector(vec.begin() + u_offset, vec.begin() + next_offset)});
                ok &= (task->add(p, fn, std::cref(type), std::ref(slices->back().sub_vec)), true);
            }
        }

        ok &= task->dispatch();

        if (ok)
        {
            for (const auto & slice : *slices)
            {
                std::copy(slice.sub_vec.begin(), slice.sub_vec.end(), vec.begin() + slice.offset);
            }
        }

        return ok ? yarp::dev::ReturnValue_ok : yarp::dev::ReturnValue_error_method_failed;
    }
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue CanBusBroker::getAvailablePids(int j, std::vector<yarp::dev::PidControlTypeEnum> & avail)
{
    CHECK_JOINT(j);
    return deviceMapper.mapSingleJoint(&yarp::dev::IPidControlRaw::getAvailablePidsRaw, j, avail);
}
#endif

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue CanBusBroker::setPid(const yarp::dev::PidControlTypeEnum & pidtype, int j, const yarp::dev::Pid & pid)
#else
bool CanBusBroker::setPid(const yarp::dev::PidControlTypeEnum & pidtype, int j, const yarp::dev::Pid & pid)
#endif
{
    CHECK_JOINT(j);
    return mapSingleJoint(deviceMapper, &yarp::dev::IPidControlRaw::setPidRaw, pidtype, j, pid);
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue CanBusBroker::setPids(const yarp::dev::PidControlTypeEnum & pidtype, const yarp::dev::Pid * pids)
#else
bool CanBusBroker::setPids(const yarp::dev::PidControlTypeEnum & pidtype, const yarp::dev::Pid * pids)
#endif
{
    return mapAllJoints(deviceMapper, &yarp::dev::IPidControlRaw::setPidsRaw, pidtype, pids);
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue CanBusBroker::setPidReference(const yarp::dev::PidControlTypeEnum & pidtype, int j, double ref)
#else
bool CanBusBroker::setPidReference(const yarp::dev::PidControlTypeEnum & pidtype, int j, double ref)
#endif
{
    CHECK_JOINT(j);
    return mapSingleJoint(deviceMapper, &yarp::dev::IPidControlRaw::setPidReferenceRaw, pidtype, j, ref);
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue CanBusBroker::setPidReferences(const yarp::dev::PidControlTypeEnum & pidtype, const double * refs)
#else
bool CanBusBroker::setPidReferences(const yarp::dev::PidControlTypeEnum & pidtype, const double * refs)
#endif
{
    return mapAllJoints(deviceMapper, &yarp::dev::IPidControlRaw::setPidReferencesRaw, pidtype, refs);
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue CanBusBroker::setPidErrorLimit(const yarp::dev::PidControlTypeEnum & pidtype, int j, double limit)
#else
bool CanBusBroker::setPidErrorLimit(const yarp::dev::PidControlTypeEnum & pidtype, int j, double limit)
#endif
{
    CHECK_JOINT(j);
    return mapSingleJoint(deviceMapper, &yarp::dev::IPidControlRaw::setPidErrorLimitRaw, pidtype, j, limit);
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue CanBusBroker::setPidErrorLimits(const yarp::dev::PidControlTypeEnum & pidtype, const double * limits)
#else
bool CanBusBroker::setPidErrorLimits(const yarp::dev::PidControlTypeEnum & pidtype, const double * limits)
#endif
{
    return mapAllJoints(deviceMapper, &yarp::dev::IPidControlRaw::setPidErrorLimitsRaw, pidtype, limits);
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue CanBusBroker::getPidError(const yarp::dev::PidControlTypeEnum & pidtype, int j, double * err)
#else
bool CanBusBroker::getPidError(const yarp::dev::PidControlTypeEnum & pidtype, int j, double * err)
#endif
{
    CHECK_JOINT(j);
    return mapSingleJoint(deviceMapper, &yarp::dev::IPidControlRaw::getPidErrorRaw, pidtype, j, err);
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue CanBusBroker::getPidErrors(const yarp::dev::PidControlTypeEnum & pidtype, double * errs)
#else
bool CanBusBroker::getPidErrors(const yarp::dev::PidControlTypeEnum & pidtype, double * errs)
#endif
{
    return mapAllJoints(deviceMapper, &yarp::dev::IPidControlRaw::getPidErrorsRaw, pidtype, errs);
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue CanBusBroker::getPidOutput(const yarp::dev::PidControlTypeEnum & pidtype, int j, double *out)
#else
bool CanBusBroker::getPidOutput(const yarp::dev::PidControlTypeEnum & pidtype, int j, double *out)
#endif
{
    CHECK_JOINT(j);
    return mapSingleJoint(deviceMapper, &yarp::dev::IPidControlRaw::getPidOutputRaw, pidtype, j, out);
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue CanBusBroker::getPidOutputs(const yarp::dev::PidControlTypeEnum & pidtype, double * outs)
#else
bool CanBusBroker::getPidOutputs(const yarp::dev::PidControlTypeEnum & pidtype, double * outs)
#endif
{
    return mapAllJoints(deviceMapper, &yarp::dev::IPidControlRaw::getPidOutputsRaw, pidtype, outs);
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue CanBusBroker::getPid(const yarp::dev::PidControlTypeEnum & pidtype, int j, yarp::dev::Pid * pid)
#else
bool CanBusBroker::getPid(const yarp::dev::PidControlTypeEnum & pidtype, int j, yarp::dev::Pid * pid)
#endif
{
    CHECK_JOINT(j);
    return mapSingleJoint(deviceMapper, &yarp::dev::IPidControlRaw::getPidRaw, pidtype, j, pid);
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue CanBusBroker::getPids(const yarp::dev::PidControlTypeEnum & pidtype, yarp::dev::Pid * pids)
#else
bool CanBusBroker::getPids(const yarp::dev::PidControlTypeEnum & pidtype, yarp::dev::Pid * pids)
#endif
{
    return mapAllJoints(deviceMapper, &yarp::dev::IPidControlRaw::getPidsRaw, pidtype, pids);
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue CanBusBroker::getPidOffset(const yarp::dev::PidControlTypeEnum & pidtype, int j, double & v)
{
    CHECK_JOINT(j);
    return mapSingleJoint(deviceMapper, &yarp::dev::IPidControlRaw::getPidOffsetRaw, pidtype, j, v);
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue CanBusBroker::getPidFeedforward(const yarp::dev::PidControlTypeEnum & pidtype, int j, double & v)
{
    CHECK_JOINT(j);
    return mapSingleJoint(deviceMapper, &yarp::dev::IPidControlRaw::getPidFeedforwardRaw, pidtype, j, v);
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue CanBusBroker::getPidExtraInfo(const yarp::dev::PidControlTypeEnum & pidtype, int j, yarp::dev::PidExtraInfo & info)
{
    CHECK_JOINT(j);
    return mapSingleJoint(deviceMapper, &yarp::dev::IPidControlRaw::getPidExtraInfoRaw, pidtype, j, info);
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue CanBusBroker::getPidExtraInfos(const yarp::dev::PidControlTypeEnum & pidtype, std::vector<yarp::dev::PidExtraInfo> & info)
{
    return mapAllJoints(deviceMapper, &yarp::dev::IPidControlRaw::getPidExtraInfosRaw, pidtype, info);
}
#endif

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue CanBusBroker::getPidReference(const yarp::dev::PidControlTypeEnum & pidtype, int j, double * ref)
#else
bool CanBusBroker::getPidReference(const yarp::dev::PidControlTypeEnum & pidtype, int j, double * ref)
#endif
{
    CHECK_JOINT(j);
    return mapSingleJoint(deviceMapper, &yarp::dev::IPidControlRaw::getPidReferenceRaw, pidtype, j, ref);
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue CanBusBroker::getPidReferences(const yarp::dev::PidControlTypeEnum & pidtype, double * refs)
#else
bool CanBusBroker::getPidReferences(const yarp::dev::PidControlTypeEnum & pidtype, double * refs)
#endif
{
    return mapAllJoints(deviceMapper, &yarp::dev::IPidControlRaw::getPidReferencesRaw, pidtype, refs);
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue CanBusBroker::getPidErrorLimit(const yarp::dev::PidControlTypeEnum & pidtype, int j, double * limit)
#else
bool CanBusBroker::getPidErrorLimit(const yarp::dev::PidControlTypeEnum & pidtype, int j, double * limit)
#endif
{
    CHECK_JOINT(j);
    return mapSingleJoint(deviceMapper, &yarp::dev::IPidControlRaw::getPidErrorLimitRaw, pidtype, j, limit);
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue CanBusBroker::getPidErrorLimits(const yarp::dev::PidControlTypeEnum & pidtype, double * limits)
#else
bool CanBusBroker::getPidErrorLimits(const yarp::dev::PidControlTypeEnum & pidtype, double * limits)
#endif
{
    return mapAllJoints(deviceMapper, &yarp::dev::IPidControlRaw::getPidErrorLimitsRaw, pidtype, limits);
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue CanBusBroker::resetPid(const yarp::dev::PidControlTypeEnum & pidtype, int j)
#else
bool CanBusBroker::resetPid(const yarp::dev::PidControlTypeEnum & pidtype, int j)
#endif
{
    CHECK_JOINT(j);
    return mapSingleJoint(deviceMapper, &yarp::dev::IPidControlRaw::resetPidRaw, pidtype, j);
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue CanBusBroker::disablePid(const yarp::dev::PidControlTypeEnum & pidtype, int j)
#else
bool CanBusBroker::disablePid(const yarp::dev::PidControlTypeEnum & pidtype, int j)
#endif
{
    CHECK_JOINT(j);
    return mapSingleJoint(deviceMapper, &yarp::dev::IPidControlRaw::disablePidRaw, pidtype, j);
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue CanBusBroker::enablePid(const yarp::dev::PidControlTypeEnum & pidtype, int j)
#else
bool CanBusBroker::enablePid(const yarp::dev::PidControlTypeEnum & pidtype, int j)
#endif
{
    CHECK_JOINT(j);
    return mapSingleJoint(deviceMapper, &yarp::dev::IPidControlRaw::enablePidRaw, pidtype, j);
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue CanBusBroker::setPidOffset(const yarp::dev::PidControlTypeEnum & pidtype, int j, double v)
#else
bool CanBusBroker::setPidOffset(const yarp::dev::PidControlTypeEnum & pidtype, int j, double v)
#endif
{
    CHECK_JOINT(j);
    return mapSingleJoint(deviceMapper, &yarp::dev::IPidControlRaw::setPidOffsetRaw, pidtype, j, v);
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue CanBusBroker::setPidFeedforward(const yarp::dev::PidControlTypeEnum & pidtype, int j, double v)
{
    CHECK_JOINT(j);
    return mapSingleJoint(deviceMapper, &yarp::dev::IPidControlRaw::setPidFeedforwardRaw, pidtype, j, v);
}
#endif

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue CanBusBroker::isPidEnabled(const yarp::dev::PidControlTypeEnum & pidtype, int j, bool & enabled)
#else
bool CanBusBroker::isPidEnabled(const yarp::dev::PidControlTypeEnum & pidtype, int j, bool * enabled)
#endif
{
    CHECK_JOINT(j);
    return mapSingleJoint(deviceMapper, &yarp::dev::IPidControlRaw::isPidEnabledRaw, pidtype, j, enabled);
}

// -----------------------------------------------------------------------------
