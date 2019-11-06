// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "FutureTask.hpp"

#include <algorithm>
#include <functional>

using namespace roboticslab;

bool FutureTask::dispatch()
{
    return std::all_of(futures.begin(), futures.end(), std::bind(&std::future<bool>::get, std::placeholders::_1));
}

std::launch SequentialTask::getPolicy() const
{
    return std::launch::deferred;
}

std::launch ParallelTask::getPolicy() const
{
    return size() >= _concurrentTasks ? std::launch::deferred : std::launch::async;
}
