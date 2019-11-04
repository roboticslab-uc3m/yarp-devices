// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __DEVICE_MAPPER_HPP__
#define __DEVICE_MAPPER_HPP__

#include <future>
#include <memory>
#include <tuple>
#include <vector>

#include <yarp/dev/PolyDriver.h>

namespace roboticslab
{

class RawDevice final
{
public:
    explicit RawDevice(yarp::dev::PolyDriver * driver);
    ~RawDevice();

    template<typename T>
    T * getHandle() const;

private:
    class Private;
    Private * priv;
};

class FutureTask
{
public:
    virtual ~FutureTask();

    template<typename T, typename Fn, typename... Args>
    void add(T * p, Fn && fn, Args &&... args);

    bool dispatch();

protected:
    virtual std::launch getPolicy() = 0;

    std::vector<std::future<bool>> futures;
};

class SequentialTask : public FutureTask
{
protected:
    virtual std::launch getPolicy() override
    { return std::launch::deferred; }
};

class ParallelTask : public FutureTask
{
public:
    ParallelTask(unsigned int concurrentTasks)
        : _concurrentTasks(concurrentTasks)
    { }

protected:
    virtual std::launch getPolicy() override
    { return std::launch::async; }

private:
    unsigned int _concurrentTasks;
};

class FutureTaskFactory
{
public:
    virtual ~FutureTaskFactory() {}
    virtual std::unique_ptr<FutureTask> createTask() = 0;
};

class SequentialTaskFactory : public FutureTaskFactory
{
public:
    virtual std::unique_ptr<FutureTask> createTask() override
    { return std::unique_ptr<SequentialTask>(new SequentialTask); }
};

class ParallelTaskFactory : public FutureTaskFactory
{
public:
    ParallelTaskFactory(unsigned int concurrentTasks)
        : _concurrentTasks(concurrentTasks)
    { }

    virtual std::unique_ptr<FutureTask> createTask() override
    { return std::unique_ptr<ParallelTask>(new ParallelTask(_concurrentTasks)); }

private:
    unsigned int _concurrentTasks;
};

class DeviceMapper
{
public:
    DeviceMapper();
    ~DeviceMapper();

    void enableParallelization(unsigned int concurrentTasks);
    bool registerDevice(yarp::dev::PolyDriver * driver);
    const RawDevice & getDevice(int deviceIndex) const;
    const RawDevice & getDevice(int globalAxis, int * localAxis) const;
    const std::vector<const RawDevice *> & getDevices() const;
    const std::vector<const RawDevice *> & getDevices(const int *& localAxisOffsets) const;
    typedef std::vector<std::tuple<const RawDevice *, int, int>> device_tuple_t;
    device_tuple_t getDevices(int globalAxesCount, const int * globalAxes) const;
    int computeLocalIndex(int globalAxis) const;
    std::vector<int> computeLocalIndices(int localAxes, const int * globalAxes, int offset) const;

    int getControlledAxes() const
    { return totalAxes; }

    template<typename T, typename... T_ref>
    using single_mapping_fn = bool (T::*)(int, T_ref...);

    template<typename T, typename... T_ref>
    bool mapSingleJoint(single_mapping_fn<T, T_ref...> fn, int j, T_ref... ref)
    {
        int localAxis;
        T * p = getDevice(j, &localAxis).getHandle<T>();
        return p ? (p->*fn)(localAxis, ref...) : false;
    }

    template<typename T, typename... T_refs>
    using full_mapping_fn = bool (T::*)(T_refs *...);

    template<typename T, typename... T_refs>
    bool mapAllJoints(full_mapping_fn<T, T_refs...> fn, T_refs *... refs)
    {
        const int * localAxisOffsets;
        const std::vector<const RawDevice *> & rawDevices = getDevices(localAxisOffsets);
        auto task = taskFactory->createTask();

        bool ok = true;

        for (int i = 0; i < rawDevices.size(); i++)
        {
            T * p = rawDevices[i]->getHandle<T>();
            ok &= p ? task->add(p, fn, refs + localAxisOffsets[i]...), true : false;
        }

        return ok && task->dispatch();
    }

    template<typename T, typename... T_refs>
    using multi_mapping_fn = bool (T::*)(int, const int *, T_refs *...);

    template<typename T, typename... T_refs>
    bool mapJointGroup(multi_mapping_fn<T, T_refs...> fn, int n_joint, const int * joints, T_refs *... refs)
    {
        auto task = taskFactory->createTask();
        bool ok = true;

        for (const auto & t : getDevices(n_joint, joints))
        {
            T * p = std::get<0>(t)->getHandle<T>();
            const auto & localIndices = computeLocalIndices(std::get<1>(t), joints, std::get<2>(t));
            ok &= p ? task->add(p, fn, std::get<1>(t), localIndices.data(), refs + std::get<2>(t)...), true : false;
        }

        return ok && task->dispatch();
    }

private:
    std::vector<const RawDevice *> rawDevices;
    std::vector<int> localAxisOffset;
    std::vector<int> rawDeviceIndexAtGlobalAxisIndex;
    int totalAxes;

    std::unique_ptr<FutureTaskFactory> taskFactory;
};

} // namespace roboticslab

#endif // __DEVICE_MAPPER_HPP__
