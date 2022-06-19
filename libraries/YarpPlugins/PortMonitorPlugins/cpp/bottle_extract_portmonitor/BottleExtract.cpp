// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "BottleExtract.hpp"

#include <algorithm>
#include <sstream>

#include <yarp/os/Bottle.h>
#include <yarp/os/Things.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/LogComponent.h>

using namespace roboticslab;

namespace
{
    YARP_LOG_COMPONENT(BE, "rl.BottleExtract")
}

const int BottleExtract::NOT_USED = -1;

bool BottleExtract::create(const yarp::os::Property& options)
{
    yCDebug(BE) << "Created!";
    yCDebug(BE) << "I am attached to the" << (options.find("sender_side").asBool() ? "sender side" : "receiver side");

    std::stringstream parsable(options.find("carrier").asString());
    std::string betweenPluses;
    std::string textForProperty;

    while (std::getline(parsable, betweenPluses, '+'))
    {
        textForProperty.append("(");
        std::replace(betweenPluses.begin(), betweenPluses.end(), '.', ' ');
        textForProperty.append(betweenPluses);
        textForProperty.append(") ");
    }

    yarp::os::Property parsed;
    parsed.fromString(textForProperty);

    if (!parsed.check("index")) // only index is mandatory
    {
        yCError(BE) << "Missing index, bye!";
        return false;
    }

    index = parsed.find("index").asInt32();
    yCDebug(BE) << "Using index:" << index;

    if (parsed.check("subindex"))
    {
        subindex = parsed.find("subindex").asInt32();
        yCDebug(BE) << "Using subindex:" << subindex;
    }
    else
    {
        subindex = NOT_USED;
        yCDebug(BE) << "Not using subindex (will not use subsubindex either)";
    }

    if (hasSubindex() && parsed.check("subsubindex"))
    {
        subsubindex = parsed.find("subsubindex").asInt32();
        yCDebug(BE) << "Using subsubindex:" << subsubindex;
    }
    else
    {
        subsubindex = NOT_USED;
        yCDebug(BE) << "Not using subsubindex";
    }

    return true;
}

void BottleExtract::destroy(void)
{
    yCDebug(BE) << "Destroyed!";
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

    if (bt == nullptr)
    {
        yCWarning(BE) << "Expected type Bottle but got wrong data type!";
        return false;
    }

    if (index >= bt->size())
    {
        yCWarning(BE) << "Index" << index << "out of range, size" << bt->size();
        return false;
    }

    if (!bt->get(index).isList())
    {
        yCWarning(BE) << "Expected list at index:" << index;
        return false;
    }

    if (hasSubindex())
    {
        yarp::os::Bottle* list = bt->get(index).asList();

        if (subindex >= list->size())
        {
            yCWarning(BE) << "Subindex" << subindex << "out of range, size" << list->size();
            return false;
        }

        if (!list->get(subindex).isList())
        {
            yCWarning(BE) << "Expected list at index, subindex:" << index << subindex;
            return false;
        }

        if (hasSubsubindex())
        {
            yarp::os::Bottle* sublist = list->get(subindex).asList();

            if (subsubindex >= sublist->size())
            {
                yCWarning(BE) << "Subsubindex" << subsubindex << "out of range, size" << sublist->size();
                return false;
            }

            if (!sublist->get(subsubindex).isList())
            {
                yCWarning(BE) << "Expected list at index, subindex, subsubindex:" << index << subindex << subsubindex;
                return false;
            }
        }
    }

    return true;
}

yarp::os::Things& BottleExtract::update(yarp::os::Things& thing)
{
    yarp::os::Bottle* bt = thing.cast_as<yarp::os::Bottle>();

    if (bt == nullptr)
    {
        yCWarning(BE) << "Expected type Bottle but got wrong data type!";
        return thing;
    }

    if (hasSubsubindex()) // from create(), this involes having a subindex too
    {
        yarp::os::Bottle* list = bt->get(index).asList();
        yarp::os::Bottle* sublist = list->get(subindex).asList();
        yarp::os::Bottle subsublistCopy;
        subsublistCopy.copy(*(sublist->get(subsubindex).asList())); // because copy constructor does not copy
        bt->clear();
        bt->append(subsublistCopy);
    }
    else if (hasSubindex())
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
