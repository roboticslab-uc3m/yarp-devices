// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __FUTURE_TASK_HPP__
#define __FUTURE_TASK_HPP__

#include <functional>
#include <memory>
#include <vector>

namespace roboticslab
{

class FutureTask
{
public:
    virtual ~FutureTask()
    { }

    template<typename T, typename Fn, typename... Args>
    void add(T * p, Fn && fn, Args &&... args)
    { deferreds.push_back([=](int) { return (p->*fn)(args...); }); }

    virtual bool dispatch() = 0;

    unsigned int size() const
    { return deferreds.size(); }

    void clear()
    { deferreds.clear(); }

protected:
    std::vector<std::function<bool(int)>> deferreds;
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
    virtual std::unique_ptr<FutureTask> createTask() override;
};

class ParallelTaskFactory : public FutureTaskFactory
{
public:
    ParallelTaskFactory(int threads);
    ~ParallelTaskFactory();

    virtual std::unique_ptr<FutureTask> createTask() override;

private:
    class Private;
    std::unique_ptr<Private> priv;
};

} // namespace roboticslab

#endif // __FUTURE_TASK_HPP__
