// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __POSITION_DIRECT_THREAD_HPP__
#define __POSITION_DIRECT_THREAD_HPP__

#include <map>
#include <set>

#include <yarp/os/Mutex.h>
#include <yarp/os/PeriodicThread.h>

#include <yarp/conf/version.h>
#if YARP_VERSION_MINOR < 2
# include <yarp/os/Vocab.h> // upstream bug, needed by the following header
#endif
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
    mutable yarp::os::Mutex mutex;
};

} // namespace roboticslab

#endif // __POSITION_DIRECT_THREAD_HPP__
