// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusLauncher.hpp"

#include <cstdio>
#include <string>
#include <sstream>
#include <vector>

#include <yarp/os/Bottle.h>
#include <yarp/os/Property.h>
#include <yarp/os/Value.h>
#include <yarp/os/Vocab.h>

#include <yarp/dev/IControlMode.h>
#include <yarp/dev/IEncoders.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/Wrapper.h>

#include <ColorDebug.h>

using namespace roboticslab;

/************************************************************************/

bool CanBusLauncher::configure(yarp::os::ResourceFinder &rf)
{
    if (rf.check("help"))
    {
        std::printf("CanBusLauncher options:\n");
        std::printf("\t--help (this help)\t--from [file.ini]\t--context [path]\t--mode [pos]\t--homePoss\n\n");
        CD_DEBUG_NO_HEADER("%s\n", rf.toString().c_str());
        return false;
    }

    yarp::conf::vocab32_t mode = rf.check("mode", yarp::os::Value(VOCAB_CM_POSITION), "initial mode of operation").asVocab();
    bool homing = rf.check("homePoss", yarp::os::Value(false), "perform initial homing procedure").asBool();

    const std::string canDevicePrefix = "devCan";
    int canDeviceId = 0;

    while (true)
    {
        std::ostringstream oss;
        oss << canDevicePrefix << canDeviceId;
        std::string canDeviceLabel = oss.str();

        yarp::os::Bottle canDeviceGroup = rf.findGroup(canDeviceLabel);

        if (canDeviceGroup.isNull())
        {
            break;
        }

        CD_DEBUG("%s\n", canDeviceGroup.toString().c_str());

        yarp::os::Property canDeviceOptions;
        canDeviceOptions.fromString(canDeviceGroup.toString());
        canDeviceOptions.put("home", homing);

        yarp::dev::PolyDriver * canDevice = new yarp::dev::PolyDriver(canDeviceOptions);

        if (!canDevice->isValid())
        {
            CD_ERROR("CAN device %s instantiation failure.\n", canDeviceLabel.c_str());
            return false;
        }

        canDevices.push(canDevice, canDeviceLabel.c_str());
        canDeviceId++;
    }

    if (canDeviceId == 0)
    {
        CD_ERROR("Empty CAN device list.\n");
        return false;
    }

    const std::string wrapperDevicePrefix = "wrapper";
    int wrapperDeviceId = 0;

    while (true)
    {
        std::ostringstream oss;
        oss << wrapperDevicePrefix << wrapperDeviceId;
        std::string wrapperDeviceLabel = oss.str();
        yarp::os::Bottle wrapperDeviceGroup = rf.findGroup(wrapperDeviceLabel);

        if (wrapperDeviceGroup.isNull())
        {
            break;
        }

        CD_DEBUG("%s\n", wrapperDeviceGroup.toString().c_str());

        yarp::os::Property wrapperDeviceOptions;
        wrapperDeviceOptions.fromString(wrapperDeviceGroup.toString());

        yarp::dev::PolyDriver * wrapperDevice = new yarp::dev::PolyDriver(wrapperDeviceOptions);

        if (!wrapperDevice->isValid())
        {
            CD_ERROR("Wrapper device %s instantiation failure.\n", wrapperDeviceLabel.c_str());
            return false;
        }

        wrapperDevices.push(wrapperDevice, wrapperDeviceLabel.c_str());

        yarp::dev::IMultipleWrapper * iMultipleWrapper;

        if (!wrapperDevice->view(iMultipleWrapper))
        {
            CD_ERROR("Unable to view IMultipleWrapper in %s.\n", wrapperDeviceLabel.c_str());
            return false;
        }

        if (!iMultipleWrapper->attachAll(canDevices))
        {
            CD_ERROR("Unable to attach CAN devices in %s.\n", wrapperDeviceLabel.c_str());
            return false;
        }

        wrapperDeviceId++;
    }

    if (wrapperDeviceId == 0)
    {
        CD_ERROR("Empty wrapper device list.\n");
        return false;
    }

    for (int i = 0; i < canDevices.size(); i++)
    {
        yarp::dev::IControlMode * iControlMode;
        yarp::dev::IEncoders * iEncoders;

        if (!canDevices[i]->poly->view(iControlMode))
        {
            CD_ERROR("Unable to view IControlMode in %s.\n", canDevices[i]->key.c_str());
            return false;
        }

        if (!canDevices[i]->poly->view(iEncoders))
        {
            CD_ERROR("Unable to view IEncoders in %s.\n", canDevices[i]->key.c_str());
            return false;
        }

        int axes;

        if (!iEncoders->getAxes(&axes))
        {
            CD_ERROR("Unable to retrieve axes in %s.\n", canDevices[i]->key.c_str());
            return false;
        }

        std::vector<yarp::conf::vocab32_t> modes(axes, mode);

        if (!iControlMode->setControlModes(modes.data()))
        {
            CD_ERROR("Unable to set %s mode in %s.\n", yarp::os::Vocab::decode(mode).c_str(), canDevices[i]->key.c_str());
            return false;
        }
    }

    return true;
}

/************************************************************************/

bool CanBusLauncher::updateModule()
{
    return true;
}

/************************************************************************/

double CanBusLauncher::getPeriod()
{
    return 3.0;
}

/************************************************************************/

bool CanBusLauncher::close()
{
    for (int i = 0; i < wrapperDevices.size(); i++)
    {
        delete wrapperDevices[i]->poly;
    }

    for (int i = 0; i < canDevices.size(); i++)
    {
        delete canDevices[i]->poly;
    }

    return true;
}

/************************************************************************/
