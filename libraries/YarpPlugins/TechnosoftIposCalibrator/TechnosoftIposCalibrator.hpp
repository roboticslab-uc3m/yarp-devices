// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __TECHNOSOFT_IPOS_CALIBRATOR_HPP__
#define __TECHNOSOFT_IPOS_CALIBRATOR_HPP__

#include <yarp/dev/CalibratorInterfaces.h>
#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/IControlMode.h>
#include <yarp/dev/IEncoders.h>
#include <yarp/dev/IPositionControl.h>
#include <yarp/dev/Wrapper.h>

namespace roboticslab
{

/**
 * @ingroup YarpPlugins
 * \defgroup TechnosoftIposCalibrator
 * @brief Contains roboticslab::TechnosoftIposCalibrator.
 */

/**
 * @ingroup TechnosoftIposCalibrator
 * @brief ...
 */
class TechnosoftIposCalibrator : public yarp::dev::DeviceDriver,
                                 public yarp::dev::IRemoteCalibrator,
                                 public yarp::dev::IWrapper
{
public:
    TechnosoftIposCalibrator()
        : axes(0), iControlMode(nullptr), iEncoders(nullptr), iPositionControl(nullptr)
    { }

    virtual bool calibrateSingleJoint(int j);
    virtual bool calibrateWholePart();
    virtual bool homingSingleJoint(int j);
    virtual bool homingWholePart();
    virtual bool parkSingleJoint(int j, bool wait);
    virtual bool parkWholePart();
    virtual bool quitCalibrate();
    virtual bool quitPark();

    virtual bool attach(yarp::dev::PolyDriver * poly);
    virtual bool detach();

    virtual bool open(yarp::os::Searchable & config);
    virtual bool close();

private:
    int axes;
    yarp::dev::IControlMode * iControlMode;
    yarp::dev::IEncoders * iEncoders;
    yarp::dev::IPositionControl * iPositionControl;
};

}  // namespace roboticslab

#endif // __TECHNOSOFT_IPOS_CALIBRATOR_HPP__
