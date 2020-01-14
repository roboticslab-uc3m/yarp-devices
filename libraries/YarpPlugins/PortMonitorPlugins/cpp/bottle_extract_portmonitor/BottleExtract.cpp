// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "BottleExtract.hpp"

#include <algorithm>
#include <sstream>

#include <yarp/os/Bottle.h>
#include <yarp/os/Things.h>
#include <yarp/os/Log.h>

namespace roboticslab
{

const int BottleExtract::NOT_USED = -1;

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

   subindex = NOT_USED;
   if(!parsed.check("subindex"))
   {
       yDebug("Not using 'subindex'\n");
   }
   else
   {
       subindex = parsed.find("subindex").asInt32();
       yDebug("Using subindex: %d\n", subindex);
   }

   subsubindex = NOT_USED;
   if(!parsed.check("subsubindex"))
   {
       yDebug("Not using 'subsubindex'\n");
   }
   else
   {
       subsubindex = parsed.find("subsubindex").asInt32();
       yDebug("Using subsubindex: %d\n", subsubindex);
   }

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
    if(index >= bt->size())
    {
        yWarning("BottleExtract: expected bigger Bottle size (%d) given used index (%d)!\n", bt->size(), index);
        return false;
    }
    if(!bt->get(index).isList())
    {
        yWarning("BottleExtract: expected list at index (%d)!\n", index);
        return false;
    }

    if(NOT_USED != subindex) // if there is a subindex
    {
        yarp::os::Bottle* list = bt->get(index).asList();
        if(subindex >= list->size())
        {
            yWarning("BottleExtract: expected bigger list size (%d) given used subindex (%d)!\n", list->size(), subindex);
            return false;
        }
        if(!list->get(subindex).isList())
        {
            yWarning("BottleExtract: expected list at index (%d) with subindex (%d)!\n", index, subindex);
            return false;
        }
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

    if(NOT_USED != subsubindex) // if there is a subsubindex
    {
        yarp::os::Bottle* list = bt->get(index).asList();
        yarp::os::Bottle* sublist = list->get(subindex).asList();
        yarp::os::Bottle subsublistCopy;
        subsublistCopy.copy(*(sublist->get(subsubindex).asList())); // because copy constructor does not copy
        bt->clear();
        bt->append(subsublistCopy);
    }
    else if(NOT_USED != subindex) // if there is a subindex
    {
        yarp::os::Bottle* list = bt->get(index).asList();
        yarp::os::Bottle sublistCopy;
        sublistCopy.copy(*(list->get(subindex).asList())); // because copy constructor does not copy
        bt->clear();
        bt->append(sublistCopy);
    }
    else // if no subindex
    {
        yarp::os::Bottle listCopy;
        listCopy.copy(*(bt->get(index).asList())); // because copy constructor does not copy
        bt->clear();
        bt->append(listCopy);
    }

    return thing;
}

void BottleExtract::trig(void)
{
}

} // namespace roboticslab
