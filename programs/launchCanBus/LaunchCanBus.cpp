// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "LaunchCanBus.hpp"

#include <cstdio>
#include <string>
#include <vector>

#include <yarp/os/Bottle.h>
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

/************************************************************************/

bool LaunchCanBus::configure(yarp::os::ResourceFinder &rf)
{
    if (rf.check("help"))
    {
        yInfo() << "LaunchCanBus options:";
        yInfo() << "\t--help (this help)\t--from [file.ini]\t--context [path]\t--mode [pos]\t--homePoss";
        yDebug() << rf.toString();
        return false;
    }

    yarp::os::Property robotConfig;
    const auto * robotConfigPtr = &robotConfig;
    std::string configPath = rf.findFileByName("config.ini");

    if (configPath.empty() || !robotConfig.fromConfigFile(configPath))
    {
        yWarning() << "Config file not found or unsufficient permissions:" << configPath;
    }

    yarp::conf::vocab32_t mode = rf.check("mode", yarp::os::Value(VOCAB_CM_POSITION), "initial mode of operation").asVocab();
    bool homing = rf.check("home", "perform initial homing procedure");

    yarp::os::Bottle devCan = rf.findGroup("devCan", "CAN controlboard devices").tail();
    yarp::os::Bottle wrapper = rf.findGroup("wrapper", "YARP wrappers devices").tail();

    if (devCan.isNull() || devCan.size() == 0)
    {
        yError() << "Missing or empty \"devCan\" section collection";
        return false;
    }

    if (wrapper.isNull() || wrapper.size() == 0)
    {
        yError() << "Missing or empty \"wrapper\" section collection";
        return false;
    }

    // CAN devices

    for (int i = 0; i < devCan.size(); i++)
    {
        std::string canDeviceLabel = devCan.get(i).asString();
        const yarp::os::Bottle & canDeviceGroup = rf.findGroup(canDeviceLabel);

        if (canDeviceGroup.isNull())
        {
            yError() << "Missing CAN device group" << canDeviceLabel;
            return false;
        }

        yarp::os::Property canDeviceOptions;
        canDeviceOptions.fromString(canDeviceGroup.toString());
        canDeviceOptions.put("robotConfig", yarp::os::Value::makeBlob(&robotConfigPtr, sizeof(robotConfigPtr)));

        yarp::dev::PolyDriver * canDevice = new yarp::dev::PolyDriver;
        canDevices.push(canDevice, canDeviceLabel.c_str());

        if (!canDevice->open(canDeviceOptions))
        {
            yError() << "CAN device" << canDeviceLabel << "configuration failure";
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
            yError() << "Missing wrapper device group" << wrapperDeviceLabel;
            return false;
        }

        yarp::os::Property wrapperDeviceOptions;
        wrapperDeviceOptions.fromString(wrapperDeviceGroup.toString());
        wrapperDeviceOptions.unput("calibrator"); // custom property added by us

        yarp::dev::PolyDriver * wrapperDevice = new yarp::dev::PolyDriver;
        wrapperDevices.push(wrapperDevice, wrapperDeviceLabel.c_str());

        if (!wrapperDevice->open(wrapperDeviceOptions))
        {
            yError() << "Wrapper device" << wrapperDeviceLabel << "configuration failure";
            return false;
        }

        yarp::dev::IMultipleWrapper * iMultipleWrapper;

        if (!wrapperDevice->view(iMultipleWrapper))
        {
            yError() << "Unable to view IMultipleWrapper in" << wrapperDeviceLabel;
            return false;
        }

        yarp::dev::PolyDriverList temp;
        temp = canDevices;

        std::string calibratorDeviceLabel = wrapperDeviceGroup.find("calibrator").asString();

        if (!calibratorDeviceLabel.empty())
        {
            if (robotConfig.toString().empty())
            {
                yWarning() << "Missing robot config, but calibrator device was requested";
                goto attachToWrapper; // ave Satanas
            }

            yarp::os::Bottle & calibratorDeviceGroup = robotConfig.findGroup(calibratorDeviceLabel);

            if (calibratorDeviceGroup.isNull())
            {
                yWarning() << "Missing calibrator device group" << calibratorDeviceLabel;
                goto attachToWrapper; // ave Satanas
            }

            yarp::os::Property calibratorDeviceOptions;
            calibratorDeviceOptions.fromString(calibratorDeviceGroup.toString());
            calibratorDeviceOptions.put("robotConfig", yarp::os::Value::makeBlob(&robotConfigPtr, sizeof(robotConfigPtr)));
            calibratorDeviceOptions.put("joints", wrapperDeviceOptions.find("joints"));

            yarp::dev::PolyDriver * calibratorDevice = new yarp::dev::PolyDriver;
            yarp::dev::PolyDriverDescriptor descriptor(calibratorDevice, "calibrator"); // key name enforced by CBW2::attachAll()
            calibratorDevices.push(descriptor);

            if (!calibratorDevice->open(calibratorDeviceOptions))
            {
                yError() << "Calibrator device" << calibratorDeviceLabel << "configuration failure";
                return false;
            }

            yarp::dev::IRemoteCalibrator * iRemoteCalibrator;

            if (!calibratorDevice->view(iRemoteCalibrator))
            {
                yError() << "Unable to view IRemoteCalibrator in calibrator device" << calibratorDeviceLabel;
                return false;
            }

            yarp::dev::IWrapper * iWrapper;

            if (!calibratorDevice->view(iWrapper))
            {
                yError() << "Unable to view IWrapper in calibrator device" << calibratorDeviceLabel;
                return false;
            }

            if (!iWrapper->attach(wrapperDevice))
            {
                yError() << "Unable to attach calibrator to wrapper device" << wrapperDeviceLabel;
                return false;
            }

            temp.push(descriptor);
        }

        attachToWrapper:
        if (!iMultipleWrapper->attachAll(temp))
        {
            yError() << "Unable to attach wrapper" << wrapperDeviceLabel << "to CAN devices";
            return false;
        }
    }

    // homing on start

    if (homing)
    {
        if (calibratorDevices.size() != 0)
        {
            for (int i = 0; i < calibratorDevices.size(); i++)
            {
                yarp::dev::IRemoteCalibrator * iRemoteCalibrator;
                calibratorDevices[i]->poly->view(iRemoteCalibrator);

                if (!iRemoteCalibrator->homingWholePart())
                {
                    yWarning() << "Homing procedure failed for calibrator id" << i;
                }
            }
        }
        else
        {
            yWarning() << "Homing procedure requested, but no calibrator devices loaded";
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
