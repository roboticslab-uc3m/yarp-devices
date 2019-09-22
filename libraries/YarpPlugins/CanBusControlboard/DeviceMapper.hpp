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

private:
    bool queryControlledAxes(const RawDevice & rd, int * axes, bool * ret);

    std::vector<RawDevice> rawDevices;
    std::vector<int> localAxisOffset;
    std::vector<int> rawDeviceIndexAtGlobalAxisIndex;
    int totalAxes;
};

} // namespace roboticslab

#endif // __DEVICE_MAPPER_HPP__
