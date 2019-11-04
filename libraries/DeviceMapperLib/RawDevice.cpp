// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "DeviceMapper.hpp"

#include <yarp/dev/ControlBoardInterfaces.h>

namespace roboticslab // gcc <7 bug
{

class RawDevice::Private
{
public:
    Private(yarp::dev::PolyDriver * driver);

    template<typename T>
    T * getHandle() const;

private:
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

template<>
inline yarp::dev::IAmplifierControlRaw * RawDevice::Private::getHandle() const
{ return iAmplifierControlRaw; }

template<>
inline yarp::dev::IAxisInfoRaw * RawDevice::Private::getHandle() const
{ return iAxisInfoRaw; }

template<>
inline yarp::dev::IControlCalibrationRaw * RawDevice::Private::getHandle() const
{ return iControlCalibrationRaw; }

template<>
inline yarp::dev::IControlLimitsRaw * RawDevice::Private::getHandle() const
{ return iControlLimitsRaw; }

template<>
inline yarp::dev::IControlModeRaw * RawDevice::Private::getHandle() const
{ return iControlModeRaw; }

template<>
inline yarp::dev::ICurrentControlRaw * RawDevice::Private::getHandle() const
{ return iCurrentControlRaw; }

template<>
inline yarp::dev::IEncodersTimedRaw * RawDevice::Private::getHandle() const
{ return iEncodersTimedRaw; }

template<>
inline yarp::dev::IImpedanceControlRaw * RawDevice::Private::getHandle() const
{ return iImpedanceControlRaw; }

template<>
inline yarp::dev::IInteractionModeRaw * RawDevice::Private::getHandle() const
{ return iInteractionModeRaw; }

template<>
inline yarp::dev::IMotorRaw * RawDevice::Private::getHandle() const
{ return iMotorRaw; }

template<>
inline yarp::dev::IMotorEncodersRaw * RawDevice::Private::getHandle() const
{ return iMotorEncodersRaw; }

template<>
inline yarp::dev::IPidControlRaw * RawDevice::Private::getHandle() const
{ return iPidControlRaw; }

template<>
inline yarp::dev::IPositionControlRaw * RawDevice::Private::getHandle() const
{ return iPositionControlRaw; }

template<>
inline yarp::dev::IPositionDirectRaw * RawDevice::Private::getHandle() const
{ return iPositionDirectRaw; }

template<>
inline yarp::dev::IPWMControlRaw * RawDevice::Private::getHandle() const
{ return iPWMControlRaw; }

template<>
inline yarp::dev::IRemoteVariablesRaw * RawDevice::Private::getHandle() const
{ return iRemoteVariablesRaw; }

template<>
inline yarp::dev::IVelocityControlRaw * RawDevice::Private::getHandle() const
{ return iVelocityControlRaw; }

template<>
inline yarp::dev::ITorqueControlRaw * RawDevice::Private::getHandle() const
{ return iTorqueControlRaw; }

RawDevice::Private::Private(yarp::dev::PolyDriver * driver)
{
    driver->view(iAmplifierControlRaw);
    driver->view(iAxisInfoRaw);
    driver->view(iControlCalibrationRaw);
    driver->view(iControlLimitsRaw);
    driver->view(iControlModeRaw);
    driver->view(iCurrentControlRaw);
    driver->view(iEncodersTimedRaw);
    driver->view(iImpedanceControlRaw);
    driver->view(iInteractionModeRaw);
    driver->view(iMotorRaw);
    driver->view(iMotorEncodersRaw);
    driver->view(iPidControlRaw);
    driver->view(iPositionControlRaw);
    driver->view(iPositionDirectRaw);
    driver->view(iPWMControlRaw);
    driver->view(iRemoteVariablesRaw);
    driver->view(iVelocityControlRaw);
    driver->view(iTorqueControlRaw);
}

RawDevice::RawDevice(yarp::dev::PolyDriver * driver)
    : priv(new Private(driver))
{ }

RawDevice::~RawDevice()
{ delete priv; }

template<typename T>
T * RawDevice::getHandle() const
{ return priv->getHandle<T>(); }

template yarp::dev::IAmplifierControlRaw * RawDevice::getHandle() const;
template yarp::dev::IAxisInfoRaw * RawDevice::getHandle() const;
template yarp::dev::IControlCalibrationRaw * RawDevice::getHandle() const;
template yarp::dev::IControlLimitsRaw * RawDevice::getHandle() const;
template yarp::dev::IControlModeRaw * RawDevice::getHandle() const;
template yarp::dev::ICurrentControlRaw * RawDevice::getHandle() const;
template yarp::dev::IEncodersTimedRaw * RawDevice::getHandle() const;
template yarp::dev::IInteractionModeRaw * RawDevice::getHandle() const;
template yarp::dev::IMotorRaw * RawDevice::getHandle() const;
template yarp::dev::IMotorEncodersRaw * RawDevice::getHandle() const;
template yarp::dev::IPidControlRaw * RawDevice::getHandle() const;
template yarp::dev::IPositionControlRaw * RawDevice::getHandle() const;
template yarp::dev::IPositionDirectRaw * RawDevice::getHandle() const;
template yarp::dev::IPWMControlRaw * RawDevice::getHandle() const;
template yarp::dev::IRemoteVariablesRaw * RawDevice::getHandle() const;
template yarp::dev::IVelocityControlRaw * RawDevice::getHandle() const;
template yarp::dev::ITorqueControlRaw * RawDevice::getHandle() const;

} // namespace roboticslab
