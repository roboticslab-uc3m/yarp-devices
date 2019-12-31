// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "FutureTask.hpp"

#include <algorithm>
#include <iterator>
#include <numeric>
#include <utility> // std::move

#include "ctpl/ctpl_stl.h"

using namespace roboticslab;

namespace roboticslab // make doxygen group these classes in the rl namespace
{

/**
 * @ingroup DeviceMapperLib
 * @brief An implementation of a sequential deferred callback executor.
 */
class SequentialTask : public FutureTask
{
public:
    virtual bool dispatch() override
    {
        return std::accumulate(deferreds.begin(), deferreds.end(), true,
                [](bool acc, const std::function<bool(int)> & fn)
                { return acc &= fn(0); });
    }
};

/**
 * @ingroup DeviceMapperLib
 * @brief An implementation of an asynchronous deferred callback executor.
 */
class ParallelTask : public FutureTask
{
public:
    ParallelTask(ctpl::thread_pool & _pool)
        : pool(_pool)
    { }

    virtual bool dispatch() override
    {
        std::vector<std::future<bool>> futures;

        std::transform(deferreds.begin(), deferreds.end(), std::back_inserter(futures),
                [this](std::function<bool(int)> & fn)
                { return pool.push(std::move(fn)); });

        return std::accumulate(futures.begin(), futures.end(), true,
                [](bool acc, std::future<bool> & f)
                { return acc &= f.get(); });
    }

private:
    ctpl::thread_pool & pool;
};

} // namespace roboticslab

class ParallelTaskFactory::Private
{
public:
    Private(int threads)
        : pool(threads)
    { }

    ctpl::thread_pool & getPool()
    { return pool; }

private:
    ctpl::thread_pool pool;
};

ParallelTaskFactory::ParallelTaskFactory(int threads)
    : priv(new Private(threads))
{ }

ParallelTaskFactory::~ParallelTaskFactory() = default;

std::unique_ptr<FutureTask> SequentialTaskFactory::createTask()
{
    return std::unique_ptr<SequentialTask>(new SequentialTask);
}

std::unique_ptr<FutureTask> ParallelTaskFactory::createTask()
{
    return std::unique_ptr<ParallelTask>(new ParallelTask(priv->getPool()));
}
