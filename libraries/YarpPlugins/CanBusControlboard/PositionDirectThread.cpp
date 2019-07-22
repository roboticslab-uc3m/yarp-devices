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
    yarp::os::LockGuard guard(mutex);

    bool hasElement = activeIds.find(j) != activeIds.end();

    if (enablePosd && !hasElement)
    {
        activeIds.insert(j);
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

    for (std::set<int>::iterator it = ids.begin(); it != ids.end(); ++it)
    {
        idToTechnosoftIpos[*it]->sendPvtTarget();
    }
}
