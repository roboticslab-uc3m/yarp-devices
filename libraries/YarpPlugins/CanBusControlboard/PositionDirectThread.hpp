// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __POSITION_DIRECT_THREAD_HPP__
#define __POSITION_DIRECT_THREAD_HPP__

#include <map>
#include <mutex>
#include <set>

#include <yarp/os/PeriodicThread.h>
#include <yarp/dev/IPositionDirect.h>

#include "ITechnosoftIpos.h"

namespace roboticslab
{

class PositionDirectThread : public yarp::os::PeriodicThread
{
public:
    PositionDirectThread(double period);
    void setNodeHandles(const std::map<int, ITechnosoftIpos *> & idToTechnosoftIpos);
    void updateControlModeRegister(int j, bool enablePosd);

protected:
    void run();

private:
    std::map<int, ITechnosoftIpos *> idToTechnosoftIpos;
    std::set<int> activeIds;
    mutable std::mutex mtx;
};

} // namespace roboticslab

#endif // __POSITION_DIRECT_THREAD_HPP__
