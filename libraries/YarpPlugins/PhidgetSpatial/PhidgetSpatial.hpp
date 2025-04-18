// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __PHIDGET_SPATIAL_HPP__
#define __PHIDGET_SPATIAL_HPP__

#include <yarp/os/all.h>
#include <yarp/dev/Drivers.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/IAnalogSensor.h>

#include <vector>
#include <math.h>

#include <phidget21.h>

/**
 * @ingroup YarpPlugins
 * @defgroup PhidgetSpatial
 * @brief Contains PhidgetSpatial.
 */

 /**
  * @ingroup PhidgetSpatial
  * @brief Implementation of a Phidgets device.
  */
class PhidgetSpatial : public yarp::dev::DeviceDriver,
                       public yarp::dev::IAnalogSensor
{
public:
    // -------- DeviceDriver declarations. Implementation in DeviceDriverImpl.cpp --------
    bool open(yarp::os::Searchable& config) override;
    bool close() override;

    //  --------- IAnalogSensor Declarations. Implementation in IAnalogSensorImpl.cpp ---------
    int read(yarp::sig::Vector &out) override;
    int getState(int ch) override;
    int getChannels() override;
    int calibrateSensor() override;
    int calibrateSensor(const yarp::sig::Vector& value) override;
    int calibrateChannel(int ch) override;
    int calibrateChannel(int ch, double value) override;

    // -- Helper Funcion declarations. Implementation in PhidgetSpatial.cpp --

    ///////////////////////////////////////////////////////////////////////////
    // The following six functions have been extracted and modified from the - Spatial simple -
    // example ((creates an Spatial handle, hooks the event handlers, and then waits for an
    // encoder is attached. Once it is attached, the program will wait for user input so that
    // we can see the event data on the screen when using the encoder. Legal info:
    // Copyright 2008 Phidgets Inc.  All rights reserved.
    // This work is licensed under the Creative Commons Attribution 2.5 Canada License.
    // view a copy of this license, visit http://creativecommons.org/licenses/by/2.5/ca/
    static int AttachHandler(CPhidgetHandle ENC, void *userptr);
    static int DetachHandler(CPhidgetHandle ENC, void *userptr);
    static int ErrorHandler(CPhidgetHandle ENC, void *userptr, int ErrorCode, const char *Description);
    static int SpatialDataHandler(CPhidgetSpatialHandle spatial, void *userptr, CPhidgetSpatial_SpatialEventDataHandle *data, int count);
    int display_properties(CPhidgetSpatialHandle phid);
    ///////////////////////////////////////////////////////////////////////////

private:
    CPhidgetSpatialHandle hSpatial0;
    yarp::os::Semaphore hSemaphore;
    double acceleration[3];
    double angularRate[3];
    double magneticField[3];
};

#endif // __PHIDGET_SPATIAL_HPP__
