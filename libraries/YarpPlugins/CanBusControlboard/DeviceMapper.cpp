// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "DeviceMapper.hpp"

#include <algorithm>
#include <functional>

#include <ColorDebug.h>

using namespace roboticslab;

FutureTask::~FutureTask()
{
    for (auto && f : futures)
    {
        f.wait();
    }
}

template<typename T, typename Fn, typename... Args>
void FutureTask::add(T * p, Fn && fn, Args &&... args)
{
    futures.push_back(std::async(getPolicy(), std::bind(fn, p), args...));
}

bool FutureTask::dispatch()
{
    return std::all_of(futures.begin(), futures.end(), std::bind(&std::future<bool>::get, std::placeholders::_1));
}

void DeviceMapper::enableParallelization(unsigned int concurrentTasks)
{
    taskFactory = std::unique_ptr<ParallelTaskFactory>(new ParallelTaskFactory(concurrentTasks));
}

bool DeviceMapper::registerDevice(yarp::dev::PolyDriver * driver)
{
    RawDevice rd;

    driver->view(rd.iAmplifierControlRaw);
    driver->view(rd.iAxisInfoRaw);
    driver->view(rd.iControlCalibrationRaw);
    driver->view(rd.iControlLimitsRaw);
    driver->view(rd.iControlModeRaw);
    driver->view(rd.iCurrentControlRaw);
    driver->view(rd.iEncodersTimedRaw);
    driver->view(rd.iImpedanceControlRaw);
    driver->view(rd.iInteractionModeRaw);
    driver->view(rd.iMotorRaw);
    driver->view(rd.iMotorEncodersRaw);
    driver->view(rd.iPidControlRaw);
    driver->view(rd.iPositionControlRaw);
    driver->view(rd.iPositionDirectRaw);
    driver->view(rd.iPWMControlRaw);
    driver->view(rd.iRemoteVariablesRaw);
    driver->view(rd.iVelocityControlRaw);
    driver->view(rd.iTorqueControlRaw);

    int axes;
    bool ret;

    if (!queryControlledAxes(rd, &axes, &ret))
    {
        CD_WARNING("Unable to get controlled axes: missing interface implementation.\n");
        return false;
    }

    if (ret)
    {
        rawDeviceIndexAtGlobalAxisIndex.insert(rawDeviceIndexAtGlobalAxisIndex.end(), axes, rawDevices.size());
        rawDevices.push_back(rd);
        localAxisOffset.push_back(totalAxes);
        totalAxes += axes;
        return true;
    }
    else
    {
        CD_WARNING("Unable to get controlled axes: query failure.\n");
        return false;
    }
}

bool DeviceMapper::queryControlledAxes(const RawDevice & rd, int * axes, bool * ret)
{
    if (rd.iCurrentControlRaw)
    {
        *ret = rd.iCurrentControlRaw->getNumberOfMotorsRaw(axes);
    }
    else if (rd.iEncodersTimedRaw)
    {
        *ret = rd.iEncodersTimedRaw->getAxes(axes);
    }
    else if (rd.iImpedanceControlRaw)
    {
        *ret = rd.iImpedanceControlRaw->getAxes(axes);
    }
    else if (rd.iMotorRaw)
    {
        *ret = rd.iMotorRaw->getNumberOfMotorsRaw(axes);
    }
    else if (rd.iMotorEncodersRaw)
    {
        *ret = rd.iMotorEncodersRaw->getNumberOfMotorEncodersRaw(axes);
    }
    else if (rd.iPositionControlRaw)
    {
        *ret = rd.iPositionControlRaw->getAxes(axes);
    }
    else if (rd.iPositionDirectRaw)
    {
        *ret = rd.iPositionDirectRaw->getAxes(axes);
    }
    else if (rd.iPWMControlRaw)
    {
        *ret = rd.iPWMControlRaw->getNumberOfMotorsRaw(axes);
    }
    else if (rd.iVelocityControlRaw)
    {
        *ret = rd.iVelocityControlRaw->getAxes(axes);
    }
    else if (rd.iTorqueControlRaw)
    {
        *ret = rd.iTorqueControlRaw->getAxes(axes);
    }
    else
    {
        return false;
    }

    return true;
}

const RawDevice & DeviceMapper::getDevice(int deviceIndex) const
{
    return rawDevices[deviceIndex];
}

const RawDevice & DeviceMapper::getDevice(int globalAxis, int * localAxis) const
{
    const int deviceIndex = rawDeviceIndexAtGlobalAxisIndex[globalAxis];
    *localAxis = globalAxis - localAxisOffset[deviceIndex];
    return rawDevices[deviceIndex];
}

const std::vector<RawDevice> & DeviceMapper::getDevices() const
{
    return rawDevices;
}

const std::vector<RawDevice> & DeviceMapper::getDevices(const int *& localAxisOffsets) const
{
    localAxisOffsets = localAxisOffset.data();
    return rawDevices;
}

DeviceMapper::device_tuple_t DeviceMapper::getDevices(int globalAxesCount, const int * globalAxes) const
{
    device_tuple_t vec;
    int previousDeviceIndex = -1;

    for (int i = 0; i < globalAxesCount; i++)
    {
        const int deviceIndex = rawDeviceIndexAtGlobalAxisIndex[globalAxes[i]];

        if (deviceIndex != previousDeviceIndex)
        {
            vec.push_back(std::make_tuple(&rawDevices[deviceIndex], 1, i));
            previousDeviceIndex = deviceIndex;
        }
        else
        {
            ++std::get<1>(vec.back());
        }
    }

    return vec;
}

int DeviceMapper::computeLocalIndex(int globalAxis) const
{
    return globalAxis - localAxisOffset[rawDeviceIndexAtGlobalAxisIndex[globalAxis]];
}

std::vector<int> DeviceMapper::computeLocalIndices(int localAxesCount, const int * globalAxes, int offset) const
{
    std::vector<int> v(localAxesCount);

    for (int i = 0; i < localAxesCount; i++)
    {
        const int globalAxis = globalAxes[i + offset];
        v[i] = globalAxis - localAxisOffset[rawDeviceIndexAtGlobalAxisIndex[globalAxis]];
    }

    return v;
}
