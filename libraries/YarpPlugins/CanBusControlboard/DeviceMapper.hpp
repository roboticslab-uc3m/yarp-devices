// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __DEVICE_MAPPER_HPP__
#define __DEVICE_MAPPER_HPP__

#include <tuple>
#include <vector>

#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>

namespace roboticslab
{

struct RawDevice
{
    yarp::dev::IAmplifierControlRaw * iAmplifierControlRaw;
    yarp::dev::IAxisInfoRaw * iAxisInfoRaw;
    yarp::dev::IControlCalibrationRaw * iControlCalibrationRaw;
    yarp::dev::IControlLimitsRaw * iControlLimitsRaw;
    yarp::dev::IControlModeRaw * iControlModeRaw;
    yarp::dev::ICurrentControlRaw * iCurrentControlRaw;
    yarp::dev::IEncodersTimedRaw * iEncodersTimedRaw;
    yarp::dev::IImpedanceControlRaw * iImpedanceControlRaw;
    yarp::dev::IInteractionModeRaw * iInteractionModeRaw;
    yarp::dev::IMotorRaw * iMotorRaw;
    yarp::dev::IMotorEncodersRaw * iMotorEncodersRaw;
    yarp::dev::IPidControlRaw * iPidControlRaw;
    yarp::dev::IPositionControlRaw * iPositionControlRaw;
    yarp::dev::IPositionDirectRaw * iPositionDirectRaw;
    yarp::dev::IPWMControlRaw * iPWMControlRaw;
    yarp::dev::IRemoteVariablesRaw * iRemoteVariablesRaw;
    yarp::dev::IVelocityControlRaw * iVelocityControlRaw;
    yarp::dev::ITorqueControlRaw * iTorqueControlRaw;

    template<typename T>
    T * getHandle() const
    { return nullptr; }
};

class DeviceMapper
{
public:
    DeviceMapper() : totalAxes(0)
    { }

    bool registerDevice(yarp::dev::PolyDriver * driver);
    const RawDevice & getDevice(int deviceIndex) const;
    const RawDevice & getDevice(int globalAxis, int * localAxis) const;
    const std::vector<RawDevice> & getDevices() const;
    const std::vector<RawDevice> & getDevices(const int *& localAxisOffsets) const;
    typedef std::vector<std::tuple<const RawDevice *, int, int>> device_tuple_t;
    device_tuple_t getDevices(int globalAxesCount, const int * globalAxes) const;
    int computeLocalIndex(int globalAxis) const;
    std::vector<int> computeLocalIndices(int localAxes, const int * globalAxes, int offset) const;

    int getControlledAxes() const
    { return totalAxes; }

    template<typename T>
    using single_mapping_fn = bool (T::*)(int, double);

    template<typename T>
    bool singleJointMapping(int j, double ref, single_mapping_fn<T> fn)
    {
        int localAxis;
        T * p = getDevice(j, &localAxis).getHandle<T>();
        return p ? (p->*fn)(localAxis, ref) : false;
    }

    template<typename T>
    using full_mapping_fn = bool (T::*)(const double *);

    template<typename T>
    bool fullJointMapping(const double * refs, full_mapping_fn<T> fn)
    {
        const int * localAxisOffsets;
        const std::vector<RawDevice> & rawDevices = getDevices(localAxisOffsets);

        bool ok = true;

        for (int i = 0; i < rawDevices.size(); i++)
        {
            T * p = rawDevices[i].getHandle<T>();
            ok &= p ? (p->*fn)(refs + localAxisOffsets[i]) : false;
        }

        return ok;
    }

    template<typename T>
    using multi_mapping_fn = bool (T::*)(int, const int *, const double *);

    template<typename T>
    bool multiJointMapping(int n_joint, const int * joints, const double * refs, multi_mapping_fn<T> fn)
    {
        bool ok = true;

        for (const auto & t : getDevices(n_joint, joints))
        {
            T * p = std::get<0>(t)->getHandle<T>();
            const auto & localIndices = computeLocalIndices(std::get<1>(t), joints, std::get<2>(t));
            ok &= p ? (p->*fn)(std::get<1>(t), localIndices.data(), refs + std::get<2>(t)) : false;
        }

        return ok;
    }

private:
    bool queryControlledAxes(const RawDevice & rd, int * axes, bool * ret);

    std::vector<RawDevice> rawDevices;
    std::vector<int> localAxisOffset;
    std::vector<int> rawDeviceIndexAtGlobalAxisIndex;
    int totalAxes;
};

} // namespace roboticslab

// template specializations
#include "DeviceMapper-inl.hpp"

#endif // __DEVICE_MAPPER_HPP__
