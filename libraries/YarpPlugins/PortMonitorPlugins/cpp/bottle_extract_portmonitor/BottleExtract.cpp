// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "BottleExtract.hpp"

#include <yarp/os/Bottle.h>
#include <yarp/os/Things.h>
#include <yarp/os/Log.h>

using namespace yarp::os;

namespace roboticslab
{

bool BottleExtract::create(const yarp::os::Property& options)
{
   yDebug("created!\n");
   yDebug("I am attached to the %s\n",
          (options.find("sender_side").asBool()) ? "sender side" : "receiver side");
   return true;
}

void BottleExtract::destroy(void)
{
    yDebug("destroyed!\n");
}

bool BottleExtract::setparam(const yarp::os::Property& params)
{
    return false;
}

bool BottleExtract::getparam(yarp::os::Property& params)
{
    return false;
}

bool BottleExtract::accept(yarp::os::Things& thing)
{   
    Bottle* bt = thing.cast_as<Bottle>();
    if(bt == NULL) {
        yWarning("BottleExtract: expected type Bottle but got wrong data type!\n");
        return false;
    }

    if(bt->toString() == "ignore")
        return false;
    return true;
}

yarp::os::Things& BottleExtract::update(yarp::os::Things& thing)
{
    Bottle* bt = thing.cast_as<Bottle>();
    if(bt == NULL) {
        yWarning("BottleExtract: expected type Bottle but got wrong data type!\n");
        return thing;
    }

    bt->addString("Modified in DLL");
    return thing;
}

void BottleExtract::trig(void)
{
}

} // namespace roboticslab
