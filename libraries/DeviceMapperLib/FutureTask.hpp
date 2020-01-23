// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __FUTURE_TASK_HPP__
#define __FUTURE_TASK_HPP__

#include <functional>
#include <memory>
#include <vector>

namespace roboticslab
{

/**
 * @ingroup DeviceMapperLib
 * @brief Base class for a deferred task executor.
 *
 * This class features a callback registry for instance methods that return a
 * boolean value. The callbacks can be dispatched on request later on.
 */
class FutureTask
{
public:
    //! Virtual destructor.
    virtual ~FutureTask() = default;

    //! Register a deferred callback given a free function.
    template<typename Fn, typename... Args>
    void add(Fn && fn, Args &&... args)
    { deferreds.push_back([=](int) { return fn(args...); }); }

    //! Register a deferred callback given a generic class instance.
    template<typename T, typename Fn, typename... Args>
    void add(T * p, Fn && fn, Args &&... args)
    { deferreds.push_back([=](int) { return (p->*fn)(args...); }); }

    //! Dispatch the registered callbacks and returns their joint result.
    virtual bool dispatch() = 0;

    //! Get the number of registered deferred callbacks.
    unsigned int size() const
    { return deferreds.size(); }

    //! Clear internal list of deferred callbacks.
    void clear()
    { deferreds.clear(); }

protected:
    std::vector<std::function<bool(int)>> deferreds;
};

/**
 * @ingroup DeviceMapperLib
 * @brief Base class for an abstract factory of @ref FutureTask.
 */
class FutureTaskFactory
{
public:
    //! Virtual destructor.
    virtual ~FutureTaskFactory() = default;

    //! Create a managed FutureTask instance.
    virtual std::unique_ptr<FutureTask> createTask() = 0;
};

/**
 * @ingroup DeviceMapperLib
 * @brief Abstract factory of @ref SequentialTask.
 */
class SequentialTaskFactory : public FutureTaskFactory
{
public:
    virtual std::unique_ptr<FutureTask> createTask() override;
};

/**
 * @ingroup DeviceMapperLib
 * @brief Abstract factory of @ref ParallelTask.
 */
class ParallelTaskFactory : public FutureTaskFactory
{
public:
    //! Constructor, creates a pool of threads.
    ParallelTaskFactory(int threads);

    virtual ~ParallelTaskFactory();

    virtual std::unique_ptr<FutureTask> createTask() override;

private:
    class Private;
    std::unique_ptr<Private> priv;
};

} // namespace roboticslab

#endif // __FUTURE_TASK_HPP__
