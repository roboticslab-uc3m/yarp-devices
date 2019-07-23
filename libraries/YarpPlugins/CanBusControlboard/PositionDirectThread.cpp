// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "PositionDirectThread.hpp"

#include <yarp/os/LockGuard.h>

using namespace roboticslab;

PositionDirectThread::PositionDirectThread(double period, int _deferIterations)
    : yarp::os::PeriodicThread(period),
      deferIterations(_deferIterations)
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
        activeIds.insert(std::make_pair(j, 0));
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
    std::map<int, int> ids = retrieveAndIncrementIds();

    for (std::map<int, int>::iterator it = ids.begin(); it != ids.end(); ++it)
    {
        if (it->second == deferIterations)
        {
            idToTechnosoftIpos[it->first]->sendLinearInterpolationStart();
        }

        idToTechnosoftIpos[it->first]->sendLinearInterpolationTarget();
    }
}

std::map<int, int> PositionDirectThread::retrieveAndIncrementIds()
{
    std::map<int, int> ids;
    yarp::os::LockGuard guard(mutex);

    for (std::map<int, int>::iterator it = activeIds.begin(); it != activeIds.end(); ++it)
    {
        ++it->second;
        ids.insert(*it);
    }

    return ids;
}
