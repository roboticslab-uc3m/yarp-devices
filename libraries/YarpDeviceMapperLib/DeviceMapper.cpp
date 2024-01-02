// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "DeviceMapper.hpp"

#include <functional> // std::function
#include <typeindex> // std::type_index
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
    using namespace yarp::dev;

    bool queryControlledAxes(const RawDevice * rd, int * axes, bool * ret)
    {
        if (auto handle = rd->getHandle<ICurrentControlRaw>(); handle != nullptr)
        {
            *ret = handle->getNumberOfMotorsRaw(axes);
        }
        else if (auto handle = rd->getHandle<IEncodersRaw>(); handle != nullptr)
        {
            *ret = handle->getAxes(axes);
        }
        else if (auto handle = rd->getHandle<IImpedanceControlRaw>(); handle != nullptr)
        {
            *ret = handle->getAxes(axes);
        }
        else if (auto handle = rd->getHandle<IMotorRaw>(); handle != nullptr)
        {
            *ret = handle->getNumberOfMotorsRaw(axes);
        }
        else if (auto handle = rd->getHandle<IMotorEncodersRaw>(); handle != nullptr)
        {
            *ret = handle->getNumberOfMotorEncodersRaw(axes);
        }
        else if (auto handle = rd->getHandle<IPositionControlRaw>(); handle != nullptr)
        {
            *ret = handle->getAxes(axes);
        }
        else if (auto handle = rd->getHandle<IPositionDirectRaw>(); handle != nullptr)
        {
            *ret = handle->getAxes(axes);
        }
        else if (auto handle = rd->getHandle<IPWMControlRaw>(); handle != nullptr)
        {
            *ret = handle->getNumberOfMotorsRaw(axes);
        }
        else if (auto handle = rd->getHandle<IVelocityControlRaw>(); handle != nullptr)
        {
            *ret = handle->getAxes(axes);
        }
        else if (auto handle = rd->getHandle<ITorqueControlRaw>(); handle != nullptr)
        {
            *ret = handle->getAxes(axes);
        }
        else
        {
            return false;
        }

        return true;
    }

    template<typename T>
    bool _queryHelper(const RawDevice * rd, const std::function<void(int, std::type_index)> & cb, std::size_t (T::*fn)() const)
    {
        if (auto handle = rd->getHandle<T>(); handle != nullptr)
        {
            if (int count = std::invoke(fn, handle); count != 0)
            {
                cb(count, typeid(T));
                return true;
            }
        }

        return false;
    }

    bool queryConnectedSensors(const RawDevice * rd, std::function<void(int, std::type_index)> cb)
    {
        bool connected = false;

        connected |= _queryHelper(rd, cb, &IThreeAxisGyroscopes::getNrOfThreeAxisGyroscopes);
        connected |= _queryHelper(rd, cb, &IThreeAxisLinearAccelerometers::getNrOfThreeAxisLinearAccelerometers);
        connected |= _queryHelper(rd, cb, &IThreeAxisMagnetometers::getNrOfThreeAxisMagnetometers);
        connected |= _queryHelper(rd, cb, &IOrientationSensors::getNrOfOrientationSensors);
        connected |= _queryHelper(rd, cb, &ITemperatureSensors::getNrOfTemperatureSensors);
        connected |= _queryHelper(rd, cb, &ISixAxisForceTorqueSensors::getNrOfSixAxisForceTorqueSensors);
        connected |= _queryHelper(rd, cb, &IContactLoadCellArrays::getNrOfContactLoadCellArrays);
        connected |= _queryHelper(rd, cb, &IEncoderArrays::getNrOfEncoderArrays);
        connected |= _queryHelper(rd, cb, &ISkinPatches::getNrOfSkinPatches);
        connected |= _queryHelper(rd, cb, &IPositionSensors::getNrOfPositionSensors);

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
