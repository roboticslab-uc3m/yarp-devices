// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "FutureTask.hpp"

#include <algorithm>
#include <functional>

using namespace roboticslab;

FutureTask::~FutureTask()
{
    for (auto && f : futures)
    {
        f.wait();
    }
}

template<typename T, typename Fn, typename... Args>
void FutureTask::add(T * p, Fn && fn, Args &&... args)
{
    futures.push_back(std::async(getPolicy(), std::bind(fn, p), args...));
}

bool FutureTask::dispatch()
{
    return std::all_of(futures.begin(), futures.end(), std::bind(&std::future<bool>::get, std::placeholders::_1));
}
