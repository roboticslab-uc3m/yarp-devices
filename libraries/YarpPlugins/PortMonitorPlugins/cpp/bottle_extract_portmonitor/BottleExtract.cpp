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
   yDebug("created!");
   yDebug("I am attached to the %s",
          (options.find("sender_side").asBool()) ? "sender side" : "receiver side");
   std::stringstream parsable(options.find("carrier").asString()); // yDebug("%s", parsable.str().c_str());
   std::string betweenPluses;
   std::string textForProperty;
   while(std::getline(parsable, betweenPluses, '+'))
   {
       textForProperty.append("(");
       std::replace(betweenPluses.begin(), betweenPluses.end(), '.', ' ');
       textForProperty.append(betweenPluses); // yDebug("** %s", betweenPluses.c_str());
       textForProperty.append(") ");
   }
   yarp::os::Property parsed;
   parsed.fromString(textForProperty); // yDebug("textForProperty: %s", textForProperty.c_str());

   if(!parsed.check("index")) // only index is mandatory
   {
       yError("Missing index, bye!");
       return false;
   }

   index = parsed.find("index").asInt32();
   yDebug("Using index: %d", index);

   if(parsed.check("subindex"))
   {
       subindex = parsed.find("subindex").asInt32();
       yDebug("Using subindex: %d", subindex);
   }
   else
   {
       subindex = NOT_USED;
       yDebug("Not using subindex (will not use subsubindex either)");
   }

   if(hasSubindex() && parsed.check("subsubindex"))
   {
       subsubindex = parsed.find("subsubindex").asInt32();
       yDebug("Using subsubindex: %d", subsubindex);
   }
   else
   {
       subsubindex = NOT_USED;
       yDebug("Not using subsubindex");
   }

   return true;
}

void BottleExtract::destroy(void)
{
    yDebug("destroyed!");
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
        yWarning("BottleExtract: expected type Bottle but got wrong data type!");
        return false;
    }
    if(index >= bt->size())
    {
        yWarning("BottleExtract: index (%d) out of range of size (%zu)!", index, bt->size());
        return false;
    }
    if(!bt->get(index).isList())
    {
        yWarning("BottleExtract: expected list at (%d)!", index);
        return false;
    }

    if(hasSubindex())
    {
        yarp::os::Bottle* list = bt->get(index).asList();
        if(subindex >= list->size())
        {
            yWarning("BottleExtract: subindex (%d) out of range of size (%zu)!", subindex, list->size());
            return false;
        }
        if(!list->get(subindex).isList())
        {
            yWarning("BottleExtract: expected list at (%d, %d)!", index, subindex);
            return false;
        }

        if(hasSubsubindex())
        {
            yarp::os::Bottle* sublist = list->get(subindex).asList();
            if(subsubindex >= sublist->size())
            {
                yWarning("BottleExtract: subsubindex (%d) out of range of size (%zu)!", subsubindex, sublist->size());
                return false;
            }
            if(!sublist->get(subsubindex).isList())
            {
                yWarning("BottleExtract: expected list at (%d, %d, %d)!", index, subindex, subsubindex);
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
        yWarning("BottleExtract: expected type Bottle but got wrong data type!");
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
