// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "DeviceMapper.hpp"

#include <yarp/dev/ControlBoardInterfaces.h>

#include <ColorDebug.h>

using namespace roboticslab;

namespace
{
    bool queryControlledAxes(const RawDevice * rd, int * axes, bool * ret)
    {
        if (rd->getHandle<yarp::dev::ICurrentControlRaw>())
        {
            *ret = rd->getHandle<yarp::dev::ICurrentControlRaw>()->getNumberOfMotorsRaw(axes);
        }
        else if (rd->getHandle<yarp::dev::IEncodersTimedRaw>())
        {
            *ret = rd->getHandle<yarp::dev::IEncodersTimedRaw>()->getAxes(axes);
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
    for (auto t : rawDevicesWithOffsets)
    {
        delete std::get<0>(t);
    }
}

void DeviceMapper::enableParallelization(unsigned int concurrentTasks)
{
    taskFactory = std::unique_ptr<ParallelTaskFactory>(new ParallelTaskFactory(concurrentTasks));
}

bool DeviceMapper::registerDevice(yarp::dev::PolyDriver * driver)
{
    RawDevice * rd = new RawDevice(driver);

    int axes;
    bool ret;

    if (!queryControlledAxes(rd, &axes, &ret))
    {
        CD_WARNING("Unable to get controlled axes: missing interface implementation.\n");
        delete rd;
        return false;
    }

    if (ret)
    {
        rawDeviceIndexAtGlobalAxisIndex.insert(rawDeviceIndexAtGlobalAxisIndex.end(), axes, rawDevicesWithOffsets.size());
        rawDevicesWithOffsets.push_back(std::make_tuple(rd, totalAxes));
        totalAxes += axes;
        return true;
    }
    else
    {
        CD_WARNING("Unable to get controlled axes: query failure.\n");
        delete rd;
        return false;
    }
}

DeviceMapper::dev_int_t DeviceMapper::getDevice(int globalAxis) const
{
    const int deviceIndex = rawDeviceIndexAtGlobalAxisIndex[globalAxis];
    const auto & t = rawDevicesWithOffsets[deviceIndex];
    return std::make_tuple(std::get<0>(t), globalAxis - std::get<1>(t));
}

const std::vector<DeviceMapper::dev_int_t> & DeviceMapper::getDevicesWithOffsets() const
{
    return rawDevicesWithOffsets;
}

std::vector<DeviceMapper::dev_int2_t> DeviceMapper::getDevices(int globalAxesCount,
        const int * globalAxes) const
{
    std::vector<dev_int2_t> vec(globalAxesCount);
    int previousDeviceIndex = -1;

    for (int i = 0; i < globalAxesCount; i++)
    {
        const int deviceIndex = rawDeviceIndexAtGlobalAxisIndex[globalAxes[i]];

        if (deviceIndex != previousDeviceIndex)
        {
            const auto & t = rawDevicesWithOffsets[deviceIndex];
            vec[i] = std::make_tuple(std::get<0>(t), 1, i);
            previousDeviceIndex = deviceIndex;
        }
        else
        {
            ++std::get<1>(vec[i]);
        }
    }

    return vec;
}

int DeviceMapper::computeLocalIndex(int globalAxis) const
{
    const auto & t = rawDevicesWithOffsets[rawDeviceIndexAtGlobalAxisIndex[globalAxis]];
    return globalAxis - std::get<1>(t);
}

std::vector<int> DeviceMapper::computeLocalIndices(int localAxesCount, const int * globalAxes, int offset) const
{
    std::vector<int> v(localAxesCount);

    for (int i = 0; i < localAxesCount; i++)
    {
        const int globalAxis = globalAxes[i + offset];
        const auto & t = rawDevicesWithOffsets[rawDeviceIndexAtGlobalAxisIndex[globalAxis]];
        v[i] = globalAxis - std::get<1>(t);
    }

    return v;
}
