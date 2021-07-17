// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __BOTTLE_EXTRACT__
#define __BOTTLE_EXTRACT__

#include <yarp/os/MonitorObject.h>

namespace roboticslab
{

class BottleExtract : public yarp::os::MonitorObject
{
public:
    bool create(const yarp::os::Property& options) override;
    void destroy(void) override;

    bool setparam(const yarp::os::Property& params) override;
    bool getparam(yarp::os::Property& params) override;

    void trig(void) override;

    bool accept(yarp::os::Things& thing) override;
    yarp::os::Things& update(yarp::os::Things& thing) override;

private:
    int index, subindex, subsubindex;
    bool hasSubindex() { return (subindex != NOT_USED); }
    bool hasSubsubindex() { return (subsubindex != NOT_USED); }

    static const int NOT_USED;
};

} // namespace roboticslab

#endif // __BOTTLE_EXTRACT__
