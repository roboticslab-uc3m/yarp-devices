// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "DumpCanBus.hpp"

#include <cstdio>
#include <ios>
#include <sstream>

#include <yarp/os/Time.h>

#include <ColorDebug.hpp>

namespace roboticslab
{

/************************************************************************/
DumpCanBus::DumpCanBus() { }

/************************************************************************/
bool DumpCanBus::configure(yarp::os::ResourceFinder &rf)
{

    if(rf.check("help"))
    {
        std::printf("DumpCanBus options:\n");
        std::printf("\t--help (this help)\t--from [file.ini]\t--context [path]\n");
        CD_DEBUG_NO_HEADER("%s\n",rf.toString().c_str());
        return false;
    }

    CD_DEBUG("%s\n",rf.toString().c_str());
    deviceDevCan0.open(rf);
    if (!deviceDevCan0.isValid())
    {
        CD_ERROR("deviceDevCan0 instantiation not worked.\n");
        return false;
    }
    deviceDevCan0.view(iCanBus);
    deviceDevCan0.view(iCanBufferFactory);

    canInputBuffer = iCanBufferFactory->createBuffer(1);

    lastNow = yarp::os::Time::now();

    return this->start();
}

/************************************************************************/

bool DumpCanBus::updateModule()
{
    //printf("DumpCanBus alive...\n");
    return true;
}

/************************************************************************/

bool DumpCanBus::close()
{
    this->stop();

    iCanBufferFactory->destroyBuffer(canInputBuffer);
    deviceDevCan0.close();

    return true;
}

/************************************************************************/

std::string DumpCanBus::msgToStr(yarp::dev::CanMessage* message)
{
    std::stringstream tmp;
    for(int i=0; i < message->getLen()-1; i++)
    {
        tmp << std::hex << static_cast<int>(message->getData()[i]) << " ";
    }
    tmp << std::hex << static_cast<int>(message->getData()[message->getLen()-1]);
    tmp << ". canId(";
    tmp << std::dec << (message->getId() & 0x7F);
    tmp << ") via(";
    tmp << std::hex << (message->getId() & 0xFF80);
    tmp << "), t:" << yarp::os::Time::now() - lastNow << "[s].";

    lastNow = yarp::os::Time::now();

    return tmp.str();
}

/************************************************************************/

}  // namespace roboticslab
