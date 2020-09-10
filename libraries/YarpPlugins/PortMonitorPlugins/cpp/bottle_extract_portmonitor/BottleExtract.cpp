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

   if(!parsed.check("index")) // only index is mandatory
   {
       yError("Missing index, bye!\n");
       return false;
   }

   index = parsed.find("index").asInt32();
   yDebug("Using index: %d", index);

   if(parsed.check("subindex"))
   {
       subindex = parsed.find("subindex").asInt32();
       yDebug("Using subindex: %d\n", subindex);
   }
   else
   {
       subindex = NOT_USED;
       yDebug("Not using subindex (will not use subsubindex either)\n");
   }

   if(hasSubindex() && parsed.check("subsubindex"))
   {
       subsubindex = parsed.find("subsubindex").asInt32();
       yDebug("Using subsubindex: %d\n", subsubindex);
   }
   else
   {
       subsubindex = NOT_USED;
       yDebug("Not using subsubindex\n");
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

    if(bt == NULL)
    {
        yWarning("BottleExtract: expected type Bottle but got wrong data type!\n");
        return false;
    }
    if(index >= bt->size())
    {
        yWarning("BottleExtract: index (%d) out of range of size (%zu)!\n", index, bt->size());
        return false;
    }
    if(!bt->get(index).isList())
    {
        yWarning("BottleExtract: expected list at (%d)!\n", index);
        return false;
    }

    if(hasSubindex())
    {
        yarp::os::Bottle* list = bt->get(index).asList();
        if(subindex >= list->size())
        {
            yWarning("BottleExtract: subindex (%d) out of range of size (%zu)!\n", subindex, list->size());
            return false;
        }
        if(!list->get(subindex).isList())
        {
            yWarning("BottleExtract: expected list at (%d, %d)!\n", index, subindex);
            return false;
        }

        if(hasSubsubindex())
        {
            yarp::os::Bottle* sublist = list->get(subindex).asList();
            if(subsubindex >= sublist->size())
            {
                yWarning("BottleExtract: subsubindex (%d) out of range of size (%zu)!\n", subsubindex, sublist->size());
                return false;
            }
            if(!sublist->get(subsubindex).isList())
            {
                yWarning("BottleExtract: expected list at (%d, %d, %d)!\n", index, subindex, subsubindex);
                return false;
            }
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

    if(hasSubsubindex()) // from create(), this involes having a subindex too
    {
        yarp::os::Bottle* list = bt->get(index).asList();
        yarp::os::Bottle* sublist = list->get(subindex).asList();
        yarp::os::Bottle subsublistCopy;
        subsublistCopy.copy(*(sublist->get(subsubindex).asList())); // because copy constructor does not copy
        bt->clear();
        bt->append(subsublistCopy);
    }
    else if(hasSubindex())
    {
        yarp::os::Bottle* list = bt->get(index).asList();
        yarp::os::Bottle sublistCopy;
        sublistCopy.copy(*(list->get(subindex).asList())); // because copy constructor does not copy
        bt->clear();
        bt->append(sublistCopy);
    }
    else
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
