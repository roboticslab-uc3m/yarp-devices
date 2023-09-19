// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
 * @ingroup yarp_devices_examples_cpp
 * @defgroup exampleRemoteJr3 exampleRemoteJr3
 * @brief This example connects to a remote Jr3 device.
 */

/**
 * @example{lineno} exampleRemoteJr3.cpp
 */

#include <yarp/os/LogStream.h>
#include <yarp/os/Network.h>
#include <yarp/os/Property.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Time.h>

#include <yarp/dev/MultipleAnalogSensorsInterfaces.h>
#include <yarp/dev/PolyDriver.h>

int main(int argc, char * argv[])
{
    yarp::os::Network yarp;

    if (!yarp::os::Network::checkNetwork())
    {
        yError() << "Please start a yarp name server first";
        return 1;
    }

    yarp::os::Property options {
        {"device", yarp::os::Value("multipleanalogsensorsclient")},
        {"remote", yarp::os::Value("/jr3")},
        {"local", yarp::os::Value("/exampleRemoteJr3")}
    };

    yarp::dev::PolyDriver device(options);

    if (!device.isValid())
    {
        yError() << "Device not available";
        return 1;
    }

    yarp::dev::ISixAxisForceTorqueSensors * sensor;

    if (!device.view(sensor))
    {
        yError() << "Unable to acquire interface";
        return 1;
    }

    int channels = sensor->getNrOfSixAxisForceTorqueSensors();
    yInfo() << "Channels:" << channels;

    for (auto ch = 0; ch < channels; ch++)
    {
        std::string name;

        if (!sensor->getSixAxisForceTorqueSensorName(ch, name))
        {
            yError() << "Unable to get name of channel" << ch;
            return 1;
        }

        yInfo() << "Channel" << ch << "has name:" << name;
    }

    int status;
    int retry = 0;
    constexpr auto MAX_RETRIES = 10;

    do
    {
        status = yarp::dev::MAS_OK; // = 0

        for (auto ch = 0; ch < channels; ch++)
        {
            status += sensor->getSixAxisForceTorqueSensorStatus(ch);
        }

        yInfo() << "Waiting for sensor to be ready... retry" << ++retry;

        if (retry >= MAX_RETRIES)
        {
            yError() << "Sensor initialization failure, max number of retries exceeded";
            return 1;
        }

        yarp::os::SystemClock::delaySystem(0.1);
    }
    while (status != yarp::dev::MAS_OK);

    int n = 0;
    constexpr auto MAX_ITERS = 500;

    yInfo() << "Performing" << MAX_ITERS << "read iterations";

    yarp::sig::Vector out;
    double timestamp;

    while (n++ < MAX_ITERS)
    {
        for (auto ch = 0; ch < channels; ch++)
        {
            if (!sensor->getSixAxisForceTorqueSensorMeasure(ch, out, timestamp))
            {
                yError() << "Unable to read channel" << ch;
                return 1;
            }

            yInfo("[%d] [%f] Channel %d: %s", n, timestamp, ch, out.toString().c_str());
        }

        yarp::os::SystemClock::delaySystem(0.01);
    }

    yInfo() << "Done";
    return 0;
}
