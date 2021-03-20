// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "DumpCanBus.hpp"

#include <ios>
#include <iomanip>
#include <iostream>

#include <yarp/os/Network.h>
#include <yarp/os/Value.h>

#include <ColorDebug.h>

using namespace roboticslab;

bool DumpCanBus::configure(yarp::os::ResourceFinder & rf)
{
    CD_DEBUG("%s\n", rf.toString().c_str());

    if (!rf.check("remote", "remote port name"))
    {
        CD_ERROR("Missing remote port name.\n");
        return false;
    }

    std::string local = rf.check("local", yarp::os::Value(DEFAULT_LOCAL_PORT), "local port name").asString();
    std::string remote = rf.find("remote").asString();
    useCanOpen = !rf.check("no-can-open");

    if (!port.open(local + "/dump:i"))
    {
        CD_ERROR("Unable to open local port.\n");
        return false;
    }

    port.setReadOnly();

    if (!yarp::os::Network::connect(remote + "/dump:o", port.getName(), "fast_tcp"))
    {
        CD_ERROR("Unable to connect to remote port.\n");
        return false;
    }

    portReader.attach(port);
    portReader.useCallback(*this);
    portReader.setStrict();

    return true;
}

bool DumpCanBus::close()
{
    portReader.interrupt();
    portReader.disableCallback();
    port.close();
    return true;
}

void DumpCanBus::onRead(yarp::os::Bottle & b)
{
    unsigned int cobId = b.get(0).asInt16();

    std::cout << std::setfill(' ');

    if (!useCanOpen)
    {
        std::cout << std::setw(3) << std::hex << cobId;
    }
    else
    {
        unsigned int id = cobId & 0x7F;

        if (id == 0)
        {
            std::cout << std::setw(10);

            switch (cobId)
            {
            case 0x00:
                std::cout << "NMT";
                break;
            case 0x80:
                std::cout << "SYNC";
                break;
            case 0x100:
                std::cout << "TIME";
                break;
            default:
                std::cout << "UNK!";
                break;
            }
        }
        else
        {
            std::cout << std::setw(3) << std::dec << id;
            std::cout << std::setw(7);

            switch (cobId - id)
            {
            case 0x80:
                std::cout << "EMCY";
                break;
            case 0x180:
                std::cout << "TPDO1";
                break;
            case 0x200:
                std::cout << "RPDO1";
                break;
            case 0x280:
                std::cout << "TPDO2";
                break;
            case 0x300:
                std::cout << "RPDO2";
                break;
            case 0x380:
                std::cout << "TPDO3";
                break;
            case 0x400:
                std::cout << "RPDO3";
                break;
            case 0x480:
                std::cout << "TPDO4";
                break;
            case 0x500:
                std::cout << "RPDO4";
                break;
            case 0x580:
                std::cout << "SDO-U";
                break;
            case 0x600:
                std::cout << "SDO-D";
                break;
            case 0x700:
                std::cout << "HRTBT";
                break;
            default:
                std::cout << "UNK!";
                break;
            }
        }
    }

    if (b.size() == 2)
    {
        yarp::os::Bottle * data = b.get(1).asList();

        std::cout << " ";
        std::cout << std::setfill('0');

        for (int i = 0; i < data->size(); i++)
        {
            std::cout << " ";
            std::cout << std::setw(2) << std::hex << (static_cast<int>(data->get(i).asInt8()) & 0xFF);
        }
    }

    std::cout << std::endl;
}
