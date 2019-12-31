// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __POSITION_DIRECT_THREAD_HPP__
#define __POSITION_DIRECT_THREAD_HPP__

#include <mutex>
#include <set>
#include <vector>

#include <yarp/os/PeriodicThread.h>
#include <yarp/os/Searchable.h>

#include <yarp/dev/IRemoteVariables.h>

#include "DeviceMapper.hpp"

namespace roboticslab
{

/**
 * @ingroup CanBusControlboard
 * @brief Periodic thread that orchestrates simultaneous target commands.
 *
 * Currently aimed for yarp::dev::IPositionDirect commands. This class aims to
 * synchronize calls across all enabled joints. Once the thread is started, it
 * bursts commands to registered raw subdevices regardless of the user actually
 * sending anything or not. See @ref LinearInterpolationBuffer.
 */
class PositionDirectThread : public yarp::os::PeriodicThread
{
public:
    //! Constructor, register subdevice interface views with provided mapper.
    PositionDirectThread(const DeviceMapper & deviceMapper);

    //! Configure this thread.
    bool configure(const yarp::os::Searchable & config);

    /**
     * @brief Manage per-joint thread start and stop given current mode of operation.
     * @param j Id of wrapped raw subdevice.
     * @param enablePosd Whether we want to enable or disable posd mode.
     */
    void updateControlModeRegister(int j, bool enablePosd);

protected:
    virtual void run() override;

private:
    std::vector<yarp::dev::IRemoteVariablesRaw *> handles;
    std::set<int> activeIds;
    mutable std::mutex mutex;
};

} // namespace roboticslab

#endif // __POSITION_DIRECT_THREAD_HPP__
