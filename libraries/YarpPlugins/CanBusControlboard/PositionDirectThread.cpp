// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "PositionDirectThread.hpp"

#include <yarp/os/LockGuard.h>

using namespace roboticslab;

PositionDirectThread::PositionDirectThread(double period)
    : yarp::os::PeriodicThread(period)
{
    suspend();
}

void PositionDirectThread::setNodeHandles(const std::map<int, ITechnosoftIpos *> & _idToTechnosoftIpos)
{
    this->idToTechnosoftIpos = _idToTechnosoftIpos;
}

void PositionDirectThread::updateControlModeRegister(int j, bool enablePosd)
{
    std::lock_guard<std::mutex> guard(mtx);

    bool hasElement = activeIds.find(j) != activeIds.end();

    if (enablePosd && !hasElement)
    {
        activeIds.insert(j);
        idToTechnosoftIpos[j]->sendLinearInterpolationStart();
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
    mtx.lock();
    std::set<int> ids = activeIds;
    mtx.unlock();

    for (std::set<int>::iterator it = ids.begin(); it != ids.end(); ++it)
    {
        idToTechnosoftIpos[*it]->sendLinearInterpolationTarget();
    }
}
