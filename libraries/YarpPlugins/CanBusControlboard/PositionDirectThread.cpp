// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "PositionDirectThread.hpp"

#include <algorithm>
#include <iterator>

#include <yarp/os/Bottle.h>

#include <ColorDebug.h>

using namespace roboticslab;

PositionDirectThread::PositionDirectThread(const DeviceMapper & deviceMapper)
    : yarp::os::PeriodicThread(0.0)
{
    const auto & devices = deviceMapper.getDevicesWithOffsets();

    std::transform(devices.cbegin(), devices.cend(), std::back_inserter(handles), [](const DeviceMapper::dev_index_t & t)
            { return std::get<0>(t)->getHandle<yarp::dev::IRemoteVariablesRaw>(); });

    suspend();
}

bool PositionDirectThread::configure(const yarp::os::Searchable & config)
{
    int periodMs = config.check("linInterpPeriodMs", yarp::os::Value(50),
            "linear interpolation mode period (milliseconds)").asInt32();

    bool ok = setPeriod(periodMs * 0.001);

    for (auto * p : handles)
    {
        if (p)
        {
            ok &= p->setRemoteVariableRaw("linInterpConfig", yarp::os::Bottle(config.toString()));
        }
    }

    return ok;
}

void PositionDirectThread::updateControlModeRegister(int j, bool enablePosd)
{
    std::lock_guard<std::mutex> guard(mutex);

    bool hasElement = activeIds.find(j) != activeIds.end();
    auto * p  = handles[j];

    if (!p)
    {
        return;
    }
    else if (enablePosd && !hasElement)
    {
        activeIds.insert(j);
        p->setRemoteVariableRaw("linInterpStart", {});
        resume();
    }
    else if (!enablePosd && hasElement)
    {
        activeIds.erase(j);

        if (activeIds.empty())
        {
            suspend();
        }
    }
}

void PositionDirectThread::run()
{
    mutex.lock();
    std::set<int> ids = activeIds;
    mutex.unlock();

    for (auto id : ids)
    {
        handles[id]->setRemoteVariableRaw("linInterpTarget", {});
    }
}
