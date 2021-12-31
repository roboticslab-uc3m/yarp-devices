// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "LaunchCanBus.hpp"

#include <cstdio>
#include <string>
#include <vector>

#include <yarp/os/Bottle.h>
#include <yarp/os/LogComponent.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Property.h>
#include <yarp/os/Value.h>
#include <yarp/os/Vocab.h>

#include <yarp/dev/CalibratorInterfaces.h>
#include <yarp/dev/IControlMode.h>
#include <yarp/dev/IEncoders.h>
#include <yarp/dev/IMultipleWrapper.h>
#include <yarp/dev/IWrapper.h>
#include <yarp/dev/PolyDriver.h>

using namespace roboticslab;

namespace
{
    YARP_LOG_COMPONENT(LCB, "rl.LaunchCanBus")
}

/************************************************************************/

bool LaunchCanBus::configure(yarp::os::ResourceFinder &rf)
{
    if (rf.check("help"))
    {
        yCInfo(LCB) << "LaunchCanBus options:";
        yCInfo(LCB) << "\t--help (this help)\t--from [file.ini]\t--context [path]\t--home";
        yCDebug(LCB) << rf.toString();
        return false;
    }

    yarp::os::Property robotConfig;
    const auto * robotConfigPtr = &robotConfig;
    std::string configPath = rf.findFileByName("config.ini");

    if (configPath.empty() || !robotConfig.fromConfigFile(configPath))
    {
        yCWarning(LCB) << "Config file not found or insufficient permissions:" << configPath;
    }

    auto mode = rf.check("mode", yarp::os::Value(VOCAB_CM_POSITION), "initial mode of operation").asVocab32();

    yarp::os::Bottle devCan = rf.findGroup("devCan", "CAN controlboard devices").tail();
    yarp::os::Bottle mapper = rf.findGroup("mapper", "YARP mapper devices").tail();
    yarp::os::Bottle wrapper = rf.findGroup("wrapper", "YARP wrappers devices").tail();

    if (devCan.isNull() || devCan.size() == 0)
    {
        yCError(LCB) << "Missing or empty \"devCan\" section collection";
        return false;
    }

    if (mapper.isNull() || mapper.size() == 0)
    {
        yCError(LCB) << "Missing or empty \"mapper\" section collection";
        return false;
    }

    if (wrapper.isNull() || wrapper.size() == 0)
    {
        yCError(LCB) << "Missing or empty \"wrapper\" section collection";
        return false;
    }

    if (mapper.size() != wrapper.size())
    {
        yCError(LCB) << "Number of mapper and wrapper devices must match: got" << mapper.size() << "vs" << wrapper.size();
        return false;
    }

    // CAN devices

    for (int i = 0; i < devCan.size(); i++)
    {
        std::string canDeviceLabel = devCan.get(i).asString();
        const yarp::os::Bottle & canDeviceGroup = rf.findGroup(canDeviceLabel);

        if (canDeviceGroup.isNull())
        {
            yCError(LCB) << "Missing CAN device group" << canDeviceLabel;
            return false;
        }

        yarp::os::Property canDeviceOptions;
        canDeviceOptions.fromString(canDeviceGroup.toString());
        canDeviceOptions.put("robotConfig", yarp::os::Value::makeBlob(&robotConfigPtr, sizeof(robotConfigPtr)));

        yarp::dev::PolyDriver * canDevice = new yarp::dev::PolyDriver;
        canDevices.push(canDevice, canDeviceLabel.c_str());

        if (!canDevice->open(canDeviceOptions))
        {
            yCError(LCB) << "CAN device" << canDeviceLabel << "configuration failure";
            return false;
        }
    }

    // joint mappers

    for (int i = 0; i < mapper.size(); i++)
    {
        std::string mapperDeviceLabel = mapper.get(i).asString();
        const yarp::os::Bottle & mapperDeviceGroup = rf.findGroup(mapperDeviceLabel);

        if (mapperDeviceGroup.isNull())
        {
            yCError(LCB) << "Missing mapper device group" << mapperDeviceLabel;
            return false;
        }

        yarp::os::Property mapperDeviceOptions;
        mapperDeviceOptions.fromString(mapperDeviceGroup.toString());
        mapperDeviceOptions.unput("calibrator"); // custom property added by us

        yarp::dev::PolyDriver * mapperDevice = new yarp::dev::PolyDriver;
        mapperDevices.push(mapperDevice, mapperDeviceLabel.c_str());

        if (!mapperDevice->open(mapperDeviceOptions))
        {
            yCError(LCB) << "Mapper device" << mapperDeviceLabel << "configuration failure";
            return false;
        }

        yarp::dev::IMultipleWrapper * iMultipleWrapper;

        if (!mapperDevice->view(iMultipleWrapper))
        {
            yCError(LCB) << "Unable to view IMultipleWrapper in" << mapperDeviceLabel;
            return false;
        }

        yarp::dev::PolyDriverList temp;
        temp = canDevices;

        std::string calibratorDeviceLabel = mapperDeviceGroup.find("calibrator").asString();

        if (!calibratorDeviceLabel.empty())
        {
            if (robotConfig.toString().empty())
            {
                yCWarning(LCB) << "Missing robot config, but calibrator device was requested";
                goto attachToMapper; // ave Satanas
            }

            yarp::os::Bottle & calibratorDeviceGroup = robotConfig.findGroup(calibratorDeviceLabel);

            if (calibratorDeviceGroup.isNull())
            {
                yCWarning(LCB) << "Missing calibrator device group" << calibratorDeviceLabel;
                goto attachToMapper; // ave Satanas
            }

            yarp::os::Property calibratorDeviceOptions;
            calibratorDeviceOptions.fromString(calibratorDeviceGroup.toString());
            calibratorDeviceOptions.put("robotConfig", yarp::os::Value::makeBlob(&robotConfigPtr, sizeof(robotConfigPtr)));
            calibratorDeviceOptions.put("joints", mapperDeviceOptions.find("joints"));

            yarp::dev::PolyDriver * calibratorDevice = new yarp::dev::PolyDriver;
            yarp::dev::PolyDriverDescriptor descriptor(calibratorDevice, "calibrator"); // key name enforced by ControlBoardRemapper::attachAllXxx()
            calibratorDevices.push(descriptor);

            if (!calibratorDevice->open(calibratorDeviceOptions))
            {
                yCError(LCB) << "Calibrator device" << calibratorDeviceLabel << "configuration failure";
                return false;
            }

            yarp::dev::IRemoteCalibrator * iRemoteCalibrator;

            if (!calibratorDevice->view(iRemoteCalibrator))
            {
                yCError(LCB) << "Unable to view IRemoteCalibrator in calibrator device" << calibratorDeviceLabel;
                return false;
            }

            yarp::dev::IWrapper * iWrapper;

            if (!calibratorDevice->view(iWrapper))
            {
                yCError(LCB) << "Unable to view IWrapper in calibrator device" << calibratorDeviceLabel;
                return false;
            }

            if (!iWrapper->attach(mapperDevice))
            {
                yCError(LCB) << "Unable to attach mapper device" << mapperDeviceLabel << "to calibrator";
                return false;
            }

            temp.push(descriptor);
        }

attachToMapper:
        if (!iMultipleWrapper->attachAll(temp))
        {
            yCError(LCB) << "Unable to attach CAN devices to mapper device" << mapperDeviceLabel;
            return false;
        }
    }

    // network wrappers

    for (int i = 0; i < wrapper.size(); i++)
    {
        std::string wrapperDeviceLabel = wrapper.get(i).asString();
        const yarp::os::Bottle & wrapperDeviceGroup = rf.findGroup(wrapperDeviceLabel);

        if (wrapperDeviceGroup.isNull())
        {
            yCError(LCB) << "Missing wrapper device group" << wrapperDeviceLabel;
            return false;
        }

        yarp::os::Property wrapperDeviceOptions;
        wrapperDeviceOptions.fromString(wrapperDeviceGroup.toString());

        yarp::dev::PolyDriver * wrapperDevice = new yarp::dev::PolyDriver;
        wrapperDevices.push(wrapperDevice, wrapperDeviceLabel.c_str());

        if (!wrapperDevice->open(wrapperDeviceOptions))
        {
            yCError(LCB) << "Wrapper device" << wrapperDeviceLabel << "configuration failure";
            return false;
        }

        yarp::dev::IWrapper * iWrapper;

        if (!wrapperDevice->view(iWrapper))
        {
            yCError(LCB) << "Unable to view IWrapper in" << wrapperDeviceLabel;
            return false;
        }

        yarp::dev::PolyDriverDescriptor * linkedMapperDevice = mapperDevices[i];

        if (!iWrapper->attach(linkedMapperDevice->poly))
        {
            yCError(LCB) << "Unable to attach mapper device" << linkedMapperDevice->key << "to wrapper device" << wrapperDeviceLabel;
            return false;
        }
    }

    // homing on start

    if (rf.check("home", "perform initial homing procedure"))
    {
        if (calibratorDevices.size() != 0)
        {
            for (int i = 0; i < calibratorDevices.size(); i++)
            {
                yarp::dev::IRemoteCalibrator * iRemoteCalibrator;
                calibratorDevices[i]->poly->view(iRemoteCalibrator);

                if (!iRemoteCalibrator->homingWholePart())
                {
                    yCWarning(LCB) << "Homing procedure failed for calibrator id" << i;
                }
            }
        }
        else
        {
            yCWarning(LCB) << "Homing procedure requested, but no calibrator devices loaded";
        }
    }

    return true;
}

/************************************************************************/

bool LaunchCanBus::updateModule()
{
    return true;
}

/************************************************************************/

double LaunchCanBus::getPeriod()
{
    return 3.0;
}

/************************************************************************/

bool LaunchCanBus::close()
{
    for (int i = 0; i < calibratorDevices.size(); i++)
    {
        if (calibratorDevices[i]->poly)
        {
            calibratorDevices[i]->poly->close();
        }

        delete calibratorDevices[i]->poly;
        calibratorDevices[i]->poly = nullptr;
    }

    for (int i = 0; i < wrapperDevices.size(); i++)
    {
        if (wrapperDevices[i]->poly)
        {
            wrapperDevices[i]->poly->close();
        }

        delete wrapperDevices[i]->poly;
        wrapperDevices[i]->poly = nullptr;
    }

    for (int i = 0; i < mapperDevices.size(); i++)
    {
        if (mapperDevices[i]->poly)
        {
            mapperDevices[i]->poly->close();
        }

        delete mapperDevices[i]->poly;
        mapperDevices[i]->poly = nullptr;
    }

    for (int i = 0; i < canDevices.size(); i++)
    {
        if (canDevices[i]->poly)
        {
            canDevices[i]->poly->close();
        }

        delete canDevices[i]->poly;
        canDevices[i]->poly = nullptr;
    }

    return true;
}

/************************************************************************/
