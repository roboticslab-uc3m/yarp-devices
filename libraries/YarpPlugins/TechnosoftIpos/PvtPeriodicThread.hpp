// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __PVT_PERIODIC_THREAD_HPP__
#define __PVT_PERIODIC_THREAD_HPP__

#include <stdint.h>

#include <yarp/os/Bottle.h>
#include <yarp/os/Mutex.h>
#include <yarp/os/PeriodicThread.h>

#include "TechnosoftIpos.hpp"

namespace roboticslab
{

class TechnosoftIpos;

/**
 * @ingroup TechnosoftIpos
 * @brief Target point in PVT interpolation mode.
 */
struct PvtPoint
{
    int t;
    double p, v;

    static PvtPoint fromBottle(const yarp::os::Bottle & b, bool hasVelocity)
    {
        PvtPoint pvtPoint;
        pvtPoint.t = b.get(0).asInt32();
        pvtPoint.p = b.get(1).asFloat64();
        pvtPoint.v = hasVelocity ? b.get(2).asFloat64() : 0.0;
        return pvtPoint;
    }

    yarp::os::Bottle toBottle() const
    {
        yarp::os::Bottle b;
        b.addInt32(t);
        b.addFloat64(p);
        b.addFloat64(v);
        return b;
    }
};

/**
 * @ingroup TechnosoftIpos
 */
class PvtPeriodicThread : public yarp::os::PeriodicThread
{
public:
    PvtPeriodicThread(double period);
    void registerDriveHandle(TechnosoftIpos * technosoftIpos);
    void setInitialPose(double target);
    void updateTarget(double target);

protected:
    void run();

private:
    void createPvtMessage(const PvtPoint & pvtPoint, uint8_t * msg);

    TechnosoftIpos * technosoftIpos;
    double period;
    double lastTargetSent;
    double lastTargetReceived;
    mutable yarp::os::Mutex mutex;
};

} // namespace roboticslab

#endif // __PVT_PERIODIC_THREAD_HPP__
