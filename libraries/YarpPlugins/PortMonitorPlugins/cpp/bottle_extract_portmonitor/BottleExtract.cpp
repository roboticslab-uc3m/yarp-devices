// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "BottleExtract.hpp"

#include <algorithm>
#include <sstream>

#include <yarp/os/Bottle.h>
#include <yarp/os/Things.h>
#include <yarp/os/Log.h>

namespace roboticslab
{

bool BottleExtract::create(const yarp::os::Property& options)
{
   yDebug("created!\n");
   yDebug("I am attached to the %s\n",
          (options.find("sender_side").asBool()) ? "sender side" : "receiver side");
   std::stringstream parsable(options.find("carrier").asString()); // yDebug("%s\n", parsable.str().c_str());
   std::string betweenPluses;
   std::string textForProperty;
   while(std::getline(parsable, betweenPluses, '+'))
   {
       textForProperty.append("(");
       std::replace(betweenPluses.begin(), betweenPluses.end(), '.', ' ');
       textForProperty.append(betweenPluses); // yDebug("** %s\n", betweenPluses.c_str());
       textForProperty.append(") ");
   }
   yarp::os::Property parsed;
   parsed.fromString(textForProperty); // yDebug("textForProperty: %s\n", textForProperty.c_str());

   if(!parsed.check("index"))
   {
       yError("Missing 'index', bye!\n");
       return false;
   }
   index = parsed.find("index").asInt32();
   yDebug("Using index: %d", index);
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
    yarp::os::Bottle* bt = thing.cast_as<yarp::os::Bottle>();
    if(bt == NULL) {
        yWarning("BottleExtract: expected type Bottle but got wrong data type!\n");
        return false;
    }

    return true;
}

yarp::os::Things& BottleExtract::update(yarp::os::Things& thing)
{
    yarp::os::Bottle* bt = thing.cast_as<yarp::os::Bottle>();
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
