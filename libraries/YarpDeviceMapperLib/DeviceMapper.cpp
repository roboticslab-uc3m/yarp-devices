// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "DeviceMapper.hpp"

#include <yarp/os/LogComponent.h>
#include <yarp/os/LogStream.h>
#include <yarp/dev/ControlBoardInterfaces.h>

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
}

DeviceMapper::DeviceMapper()
    : totalAxes(0), taskFactory(new SequentialTaskFactory)
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
    const auto * rd = new RawDevice(driver);

    int axes;
    bool ret;

    if (!queryControlledAxes(rd, &axes, &ret))
    {
        yCWarning(DM) << "Unable to get controlled axes: missing interface implementation";
        delete rd;
        return false;
    }

    if (ret)
    {
        rawDeviceIndexAtGlobalAxisIndex.insert(rawDeviceIndexAtGlobalAxisIndex.end(), axes, rawDevicesWithOffsets.size());
        rawDevicesWithOffsets.emplace_back(rd, totalAxes);
        totalAxes += axes;
        return true;
    }
    else
    {
        yCWarning(DM) << "Unable to get controlled axes: query failure";
        delete rd;
        return false;
    }
}

DeviceMapper::dev_index_t DeviceMapper::getDevice(int globalAxis) const
{
    int deviceIndex = rawDeviceIndexAtGlobalAxisIndex[globalAxis];
    const auto & [rawDevice, offset] = rawDevicesWithOffsets[deviceIndex];
    return {rawDevice, globalAxis - offset};
}

const std::vector<DeviceMapper::dev_index_t> & DeviceMapper::getDevicesWithOffsets() const
{
    return rawDevicesWithOffsets;
}

std::vector<DeviceMapper::dev_group_t> DeviceMapper::getDevices(int globalAxesCount, const int * globalAxes) const
{
    std::vector<dev_group_t> vec;
    int previousDeviceIndex = -1;

    for (int i = 0; i < globalAxesCount; i++)
    {
        const int globalAxis = globalAxes[i];
        const int deviceIndex = rawDeviceIndexAtGlobalAxisIndex[globalAxis];
        const auto & [rawDevice, offset] = rawDevicesWithOffsets[deviceIndex];
        const int localIndex = globalAxis - offset;

        if (deviceIndex != previousDeviceIndex)
        {
            vec.emplace_back(rawDevice, std::vector<int>{localIndex}, i);
            previousDeviceIndex = deviceIndex;
        }
        else
        {
            std::get<1>(vec.back()).push_back(localIndex);
        }
    }

    return vec;
}

void DeviceMapper::clear()
{
    for (const auto & [device, offset] : rawDevicesWithOffsets)
    {
        delete device;
    }

    rawDevicesWithOffsets.clear();
    rawDeviceIndexAtGlobalAxisIndex.clear();
    totalAxes = 0;
}
