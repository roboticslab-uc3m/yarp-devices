// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __BOTTLE_EXTRACT__
#define __BOTTLE_EXTRACT__

#include <yarp/os/MonitorObject.h>

namespace roboticslab
{

class BottleExtract : public yarp::os::MonitorObject
{
public:
    bool create(const yarp::os::Property& options);
    void destroy(void);

    bool setparam(const yarp::os::Property& params);
    bool getparam(yarp::os::Property& params);

    void trig(void);

    bool accept(yarp::os::Things& thing);
    yarp::os::Things& update(yarp::os::Things& thing);

private:
    int index, subindex;

    static const int NOT_USED;
};

} // namespace roboticslab

#endif // __BOTTLE_EXTRACT__
