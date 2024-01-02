// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
 * @ingroup yarp_devices_examples_cpp
 * @defgroup exampleJr3Pci exampleJr3Pci
 * @brief This example instantiates a local @ref Jr3Pci device.
 */

/**
 * @example{lineno} exampleJr3Pci.cpp
 */

#include <yarp/os/LogStream.h>
#include <yarp/os/Property.h>
#include <yarp/os/SystemClock.h>

#include <yarp/dev/MultipleAnalogSensorsInterfaces.h>
#include <yarp/dev/PolyDriver.h>

int main(int argc, char * argv[])
{
    yarp::os::Property options {{"device", yarp::os::Value("Jr3Pci")}};

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
        sensor->getSixAxisForceTorqueSensorName(ch, name);
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
