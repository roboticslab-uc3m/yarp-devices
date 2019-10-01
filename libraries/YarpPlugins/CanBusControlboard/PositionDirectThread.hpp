// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __POSITION_DIRECT_THREAD_HPP__
#define __POSITION_DIRECT_THREAD_HPP__

#include <mutex>
#include <set>

#include <yarp/os/PeriodicThread.h>

#include "DeviceMapper.hpp"

namespace roboticslab
{

class PositionDirectThread : public yarp::os::PeriodicThread
{
public:
    PositionDirectThread(const DeviceMapper & deviceMapper, double period);
    void updateControlModeRegister(int j, bool enablePosd);

protected:
    void run();

private:
    const DeviceMapper & deviceMapper;
    std::set<int> activeIds;
    mutable std::mutex mutex;
};

} // namespace roboticslab

#endif // __POSITION_DIRECT_THREAD_HPP__
