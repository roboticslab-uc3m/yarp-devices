// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "PositionDirectThread.hpp"

#include <algorithm>
#include <iterator>

#include <yarp/os/Bottle.h>
#include <yarp/os/Time.h>

#include <yarp/dev/IControlMode.h>

#include <ColorDebug.h>

using namespace roboticslab;

PositionDirectThread::PositionDirectThread(const DeviceMapper & deviceMapper)
    : yarp::os::PeriodicThread(0.0)
{
    const auto & devices = deviceMapper.getDevicesWithOffsets();

    std::transform(devices.cbegin(), devices.cend(), std::back_inserter(handles), [](const DeviceMapper::dev_index_t & t)
            { return std::get<0>(t)->getHandle<yarp::dev::IRemoteVariablesRaw>(); });
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

bool PositionDirectThread::updateControlModeRegister(int j, int mode)
{
    return updateControlModeRegister({{j, mode == VOCAB_CM_POSITION_DIRECT}});
}

bool PositionDirectThread::updateControlModeRegister(int * modes, int size)
{
    std::vector<std::pair<int, bool>> ctrl;

    for (int i = 0; i < size; i++)
    {
        ctrl.push_back({i, modes[i] == VOCAB_CM_POSITION_DIRECT});
    }

    return updateControlModeRegister(ctrl);
}

bool PositionDirectThread::updateControlModeRegister(int n_joint, const int * joints, int * modes)
{
    std::vector<std::pair<int, bool>> ctrl;

    for (int i = 0; i < n_joint; i++)
    {
        ctrl.push_back({joints[i], modes[i] == VOCAB_CM_POSITION_DIRECT});
    }

    return updateControlModeRegister(ctrl);
}

bool PositionDirectThread::updateControlModeRegister(const std::vector<std::pair<int, bool>> & ctrl)
{
    for (const auto & el : ctrl)
    {
        auto * p  = handles[el.first];

        if (!p)
        {
            return false;
        }

        std::lock_guard<std::mutex> guard(mutex);

        if (el.second)
        {
            if (activeIds.insert(el.first).second)
            {
                // if a new id has been registered, send start command
                p->setRemoteVariableRaw("linInterpStart", {});
            }
        }
        else
        {
            activeIds.erase(el.first);
        }
    }

    if (isRunning() && activeIds.empty())
    {
        askToStop();
    }
    else if (!isRunning() && !activeIds.empty())
    {
        return start();
    }

    return true;
}

bool PositionDirectThread::threadInit()
{
    // wait a bit, then start bursting targets at a fixed period
    yarp::os::Time::delay(getPeriod() * 0.5);
    return true;
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
