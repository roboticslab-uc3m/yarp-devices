// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __FUTURE_TASK_HPP__
#define __FUTURE_TASK_HPP__

#include <future>
#include <memory>
#include <vector>

namespace roboticslab
{

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

} // namespace roboticslab

#endif // __FUTURE_TASK_HPP__
