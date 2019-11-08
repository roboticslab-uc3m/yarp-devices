// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __DEVICE_MAPPER_HPP__
#define __DEVICE_MAPPER_HPP__

#include <memory>
#include <tuple>
#include <vector>

#include <yarp/dev/PolyDriver.h>

#include "FutureTask.hpp"

namespace roboticslab
{

class RawDevice final
{
public:
    explicit RawDevice(yarp::dev::PolyDriver * driver);
    virtual ~RawDevice();

    RawDevice(const RawDevice &) = delete;
    RawDevice & operator=(const RawDevice &) = delete;

    template<typename T>
    T * getHandle() const;

private:
    class Private;
    Private * priv;
};

class DeviceMapper final
{
public:
    DeviceMapper();
    ~DeviceMapper();

    void enableParallelization(unsigned int concurrentTasks);
    bool registerDevice(yarp::dev::PolyDriver * driver);

    using dev_index_t = std::tuple<const RawDevice *, int>;
    using dev_group_t = std::tuple<const RawDevice *, std::vector<int>, int>;

    dev_index_t getDevice(int globalAxis) const;
    const std::vector<dev_index_t> & getDevicesWithOffsets() const;
    std::vector<dev_group_t> getDevices(int globalAxesCount, const int * globalAxes) const;

    int getControlledAxes() const
    { return totalAxes; }

    std::unique_ptr<FutureTask> createTask() const
    { return taskFactory->createTask(); }

    template<typename T, typename... T_ref>
    using single_mapping_fn = bool (T::*)(int, T_ref...);

    template<typename T, typename... T_ref>
    bool mapSingleJoint(single_mapping_fn<T, T_ref...> fn, int j, T_ref... ref)
    {
        auto t = getDevice(j);
        T * p = std::get<0>(t)->getHandle<T>();
        return p ? (p->*fn)(std::get<1>(t), ref...) : false;
    }

    template<typename T, typename... T_refs>
    using full_mapping_fn = bool (T::*)(T_refs *...);

    template<typename T, typename... T_refs>
    bool mapAllJoints(full_mapping_fn<T, T_refs...> fn, T_refs *... refs)
    {
        auto task = createTask();
        bool ok = true;

        for (const auto & t : getDevicesWithOffsets())
        {
            T * p = std::get<0>(t)->getHandle<T>();
            ok &= p ? task->add(p, fn, refs + std::get<1>(t)...), true : false;
        }

        return ok && task->dispatch();
    }

    template<typename T, typename... T_refs>
    using multi_mapping_fn = bool (T::*)(int, const int *, T_refs *...);

    template<typename T, typename... T_refs>
    bool mapJointGroup(multi_mapping_fn<T, T_refs...> fn, int n_joint, const int * joints, T_refs *... refs)
    {
        auto task = createTask();
        auto devices = getDevices(n_joint, joints); // extend lifetime of local joint vector
        bool ok = true;

        for (const auto & t : devices)
        {
            T * p = std::get<0>(t)->getHandle<T>();
            ok &= p ? task->add(p, fn, std::get<1>(t).size(), std::get<1>(t).data(), refs + std::get<2>(t)...), true : false;
        }

        return ok && task->dispatch();
    }

private:
    std::vector<dev_index_t> rawDevicesWithOffsets;
    std::vector<int> rawDeviceIndexAtGlobalAxisIndex;
    int totalAxes;

    std::unique_ptr<FutureTaskFactory> taskFactory;
};

} // namespace roboticslab

#endif // __DEVICE_MAPPER_HPP__
