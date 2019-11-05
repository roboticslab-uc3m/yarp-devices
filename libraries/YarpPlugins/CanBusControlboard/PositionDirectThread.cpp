// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "PositionDirectThread.hpp"

#include <algorithm>
#include <memory>

#include <yarp/dev/IRemoteVariables.h>

using namespace roboticslab;

PositionDirectThread::PositionDirectThread(const DeviceMapper & _deviceMapper, double period)
    : yarp::os::PeriodicThread(period),
      deviceMapper(_deviceMapper)
{
    suspend();
}

void PositionDirectThread::updateControlModeRegister(int j, bool enablePosd)
{
    std::lock_guard<std::mutex> guard(mutex);

    bool hasElement = activeIds.find(j) != activeIds.end();

    int localAxis;
    auto t = deviceMapper.getDevice(j);
    auto * p  = std::get<0>(t)->getHandle<yarp::dev::IRemoteVariablesRaw>();

    if (!p)
    {
        return;
    }
    else if (enablePosd && !hasElement)
    {
        activeIds.insert(j);
        p->setRemoteVariableRaw("linInterpStart", yarp::os::Bottle());
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

    std::unique_ptr<int[]> arr(new int[ids.size()]);
    std::copy(ids.begin(), ids.end(), arr.get());

    for (const auto & t : deviceMapper.getDevices(ids.size(), arr.get()))
    {
        auto * p = std::get<0>(t)->getHandle<yarp::dev::IRemoteVariablesRaw>();
        p->setRemoteVariableRaw("linInterpTarget", yarp::os::Bottle());
    }
}
