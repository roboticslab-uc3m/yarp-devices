// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "DeviceMapper.hpp"

#include <yarp/dev/ControlBoardInterfaces.h>

using namespace roboticslab;

class RawDevice::Private
{
public:
    Private(yarp::dev::PolyDriver * driver);

    explicit operator yarp::dev::IAmplifierControlRaw *() const
    { return iAmplifierControlRaw; }

    explicit operator yarp::dev::IAxisInfoRaw *() const
    { return iAxisInfoRaw; }

    explicit operator yarp::dev::IControlCalibrationRaw *() const
    { return iControlCalibrationRaw; }

    explicit operator yarp::dev::IControlLimitsRaw *() const
    { return iControlLimitsRaw; }

    explicit operator yarp::dev::IControlModeRaw *() const
    { return iControlModeRaw; }

    explicit operator yarp::dev::ICurrentControlRaw *() const
    { return iCurrentControlRaw; }

    explicit operator yarp::dev::IEncodersTimedRaw *() const
    { return iEncodersTimedRaw; }

    explicit operator yarp::dev::IImpedanceControlRaw *() const
    { return iImpedanceControlRaw; }

    explicit operator yarp::dev::IInteractionModeRaw *() const
    { return iInteractionModeRaw; }

    explicit operator yarp::dev::IMotorRaw *() const
    { return iMotorRaw; }

    explicit operator yarp::dev::IMotorEncodersRaw *() const
    { return iMotorEncodersRaw; }

    explicit operator yarp::dev::IPidControlRaw *() const
    { return iPidControlRaw; }

    explicit operator yarp::dev::IPositionControlRaw *() const
    { return iPositionControlRaw; }

    explicit operator yarp::dev::IPositionDirectRaw *() const
    { return iPositionDirectRaw; }

    explicit operator yarp::dev::IPWMControlRaw *() const
    { return iPWMControlRaw; }

    explicit operator yarp::dev::IRemoteVariablesRaw *() const
    { return iRemoteVariablesRaw; }

    explicit operator yarp::dev::IVelocityControlRaw *() const
    { return iVelocityControlRaw; }

    explicit operator yarp::dev::ITorqueControlRaw *() const
    { return iTorqueControlRaw; }

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
