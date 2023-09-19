// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "DeviceMapper.hpp"

#include <functional> // std::function
#include <utility> // std::move

#include <yarp/os/LogComponent.h>
#include <yarp/os/LogStream.h>

#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/MultipleAnalogSensorsInterfaces.h>

using namespace roboticslab;

namespace
{
    YARP_LOG_COMPONENT(DM, "rl.DeviceMapper")
}

namespace
{
    bool queryControlledAxes(const RawDevice * rd, int * axes, bool * ret)
    {
        if (rd->getHandle<yarp::dev::ICurrentControlRaw>())
        {
            *ret = rd->getHandle<yarp::dev::ICurrentControlRaw>()->getNumberOfMotorsRaw(axes);
        }
        else if (rd->getHandle<yarp::dev::IEncodersRaw>())
        {
            *ret = rd->getHandle<yarp::dev::IEncodersRaw>()->getAxes(axes);
        }
        else if (rd->getHandle<yarp::dev::IImpedanceControlRaw>())
        {
            *ret = rd->getHandle<yarp::dev::IImpedanceControlRaw>()->getAxes(axes);
        }
        else if (rd->getHandle<yarp::dev::IMotorRaw>())
        {
            *ret = rd->getHandle<yarp::dev::IMotorRaw>()->getNumberOfMotorsRaw(axes);
        }
        else if (rd->getHandle<yarp::dev::IMotorEncodersRaw>())
        {
            *ret = rd->getHandle<yarp::dev::IMotorEncodersRaw>()->getNumberOfMotorEncodersRaw(axes);
        }
        else if (rd->getHandle<yarp::dev::IPositionControlRaw>())
        {
            *ret = rd->getHandle<yarp::dev::IPositionControlRaw>()->getAxes(axes);
        }
        else if (rd->getHandle<yarp::dev::IPositionDirectRaw>())
        {
            *ret = rd->getHandle<yarp::dev::IPositionDirectRaw>()->getAxes(axes);
        }
        else if (rd->getHandle<yarp::dev::IPWMControlRaw>())
        {
            *ret = rd->getHandle<yarp::dev::IPWMControlRaw>()->getNumberOfMotorsRaw(axes);
        }
        else if (rd->getHandle<yarp::dev::IVelocityControlRaw>())
        {
            *ret = rd->getHandle<yarp::dev::IVelocityControlRaw>()->getAxes(axes);
        }
        else if (rd->getHandle<yarp::dev::ITorqueControlRaw>())
        {
            *ret = rd->getHandle<yarp::dev::ITorqueControlRaw>()->getAxes(axes);
        }
        else
        {
            return false;
        }

        return true;
    }

    bool queryConnectedSensors(const RawDevice * rd, std::function<void(int, std::type_index)> fn)
    {
        bool connected = false;

        if (rd->getHandle<yarp::dev::IThreeAxisGyroscopes>())
        {
            int count = rd->getHandle<yarp::dev::IThreeAxisGyroscopes>()->getNrOfThreeAxisGyroscopes();
            fn(count, std::type_index(typeid(yarp::dev::IThreeAxisGyroscopes)));
            connected = true;
        }

        if (rd->getHandle<yarp::dev::IThreeAxisLinearAccelerometers>())
        {
            int count = rd->getHandle<yarp::dev::IThreeAxisLinearAccelerometers>()->getNrOfThreeAxisLinearAccelerometers();
            fn(count, std::type_index(typeid(yarp::dev::IThreeAxisLinearAccelerometers)));
            connected = true;
        }

        if (rd->getHandle<yarp::dev::IThreeAxisMagnetometers>())
        {
            int count = rd->getHandle<yarp::dev::IThreeAxisMagnetometers>()->getNrOfThreeAxisMagnetometers();
            fn(count, std::type_index(typeid(yarp::dev::IThreeAxisMagnetometers)));
            connected = true;
        }

        if (rd->getHandle<yarp::dev::IOrientationSensors>())
        {
            int count = rd->getHandle<yarp::dev::IOrientationSensors>()->getNrOfOrientationSensors();
            fn(count, std::type_index(typeid(yarp::dev::IOrientationSensors)));
            connected = true;
        }

        if (rd->getHandle<yarp::dev::ITemperatureSensors>())
        {
            int count = rd->getHandle<yarp::dev::ITemperatureSensors>()->getNrOfTemperatureSensors();
            fn(count, std::type_index(typeid(yarp::dev::ITemperatureSensors)));
            connected = true;
        }

        if (rd->getHandle<yarp::dev::ISixAxisForceTorqueSensors>())
        {
            int count = rd->getHandle<yarp::dev::ISixAxisForceTorqueSensors>()->getNrOfSixAxisForceTorqueSensors();
            fn(count, std::type_index(typeid(yarp::dev::ISixAxisForceTorqueSensors)));
            connected = true;
        }

        if (rd->getHandle<yarp::dev::IContactLoadCellArrays>())
        {
            int count = rd->getHandle<yarp::dev::IContactLoadCellArrays>()->getNrOfContactLoadCellArrays();
            fn(count, std::type_index(typeid(yarp::dev::IContactLoadCellArrays)));
            connected = true;
        }

        if (rd->getHandle<yarp::dev::IEncoderArrays>())
        {
            int count = rd->getHandle<yarp::dev::IEncoderArrays>()->getNrOfEncoderArrays();
            fn(count, std::type_index(typeid(yarp::dev::IEncoderArrays)));
            connected = true;
        }

        if (rd->getHandle<yarp::dev::ISkinPatches>())
        {
            int count = rd->getHandle<yarp::dev::ISkinPatches>()->getNrOfSkinPatches();
            fn(count, std::type_index(typeid(yarp::dev::ISkinPatches)));
            connected = true;
        }

        if (rd->getHandle<yarp::dev::IPositionSensors>())
        {
            int count = rd->getHandle<yarp::dev::IPositionSensors>()->getNrOfPositionSensors();
            fn(count, std::type_index(typeid(yarp::dev::IPositionSensors)));
            connected = true;
        }

        return connected;
    }
}

DeviceMapper::DeviceMapper()
    : taskFactory(new SequentialTaskFactory)
{ }

DeviceMapper::~DeviceMapper()
{
    clear();
}

void DeviceMapper::enableParallelization(unsigned int concurrentTasks)
{
    taskFactory = std::make_unique<ParallelTaskFactory>(concurrentTasks);
}

bool DeviceMapper::registerDevice(yarp::dev::PolyDriver * driver)
{
    auto rd = std::make_unique<const RawDevice>(driver);

    bool ret;
    int localAxes;
    bool isMotorDevice = queryControlledAxes(rd.get(), &localAxes, &ret);

    if (isMotorDevice)
    {
        if (!ret)
        {
            yCWarning(DM) << "Motor interface detected, but unable to query controlled axes";
            return false;
        }

        motorOffsets.insert(motorOffsets.end(), localAxes, {devices.size(), totalAxes, localAxes});
        totalAxes += localAxes;
    }

    bool isSensorDevice = queryConnectedSensors(rd.get(), [this](int count, std::type_index hashable)
    {
        sensorOffsets[hashable].insert(sensorOffsets[hashable].end(), count, {devices.size(), connectedSensors[hashable], count});
        connectedSensors[hashable] += count;
    });

    if (isMotorDevice || isSensorDevice)
    {
        devices.emplace_back(std::move(rd));
        return true;
    }

    yCWarning(DM) << "Device does not implement any supported interfaces";
    return false;
}

DeviceMapper::dev_index_t DeviceMapper::getMotorDevice(int globalAxis) const
{
    const auto & [deviceIndex, offset, axes] = motorOffsets[globalAxis];
    return {devices[deviceIndex].get(), globalAxis - offset};
}

std::vector<DeviceMapper::dev_index_t> DeviceMapper::getMotorDevicesWithOffsets() const
{
    std::vector<dev_index_t> out;

    for (int i = 0; i < totalAxes; /**/)
    {
        const auto & [deviceIndex, offset, axes] = motorOffsets[i];
        out.emplace_back(devices[deviceIndex].get(), offset);
        i += axes;
    }

    return out;
}

std::vector<DeviceMapper::dev_group_t> DeviceMapper::getMotorDevicesWithIndices(int globalAxesCount, const int * globalAxes) const
{
    std::vector<dev_group_t> out;
    int previousDeviceIndex = -1;

    for (int i = 0; i < globalAxesCount; i++)
    {
        const int globalAxis = globalAxes[i];
        const auto & [deviceIndex, offset, localAxes] = motorOffsets[globalAxis];
        const int localIndex = globalAxis - offset;

        if (deviceIndex != previousDeviceIndex)
        {
            out.emplace_back(devices[deviceIndex].get(), std::vector<int>{localIndex}, i);
            previousDeviceIndex = deviceIndex;
        }
        else
        {
            std::get<1>(out.back()).push_back(localIndex);
        }
    }

    return out;
}

void DeviceMapper::clear()
{
    devices.clear();
    motorOffsets.clear();
    sensorOffsets.clear();
    connectedSensors.clear();
    totalAxes = 0;
}

const int DeviceMapper::getSensorFailureStatus()
{
    return yarp::dev::MAS_UNKNOWN;
}
