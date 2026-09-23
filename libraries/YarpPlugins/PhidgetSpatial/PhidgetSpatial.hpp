// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __PHIDGET_SPATIAL_HPP__
#define __PHIDGET_SPATIAL_HPP__

#include <mutex>

#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/MultipleAnalogSensorsInterfaces.h>

#include <phidget21.h>

constexpr auto NUM_SENSORS = 1;

#define CHECK_SENSOR(n) do { if ((n) < 0 || (n) > NUM_SENSORS - 1) return false; } while (0)

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
                       public yarp::dev::IThreeAxisLinearAccelerometers,
                       public yarp::dev::IThreeAxisGyroscopes,
                       public yarp::dev::IThreeAxisMagnetometers
{
public:
    // -------- DeviceDriver declarations. Implementation in DeviceDriverImpl.cpp --------
    bool open(yarp::os::Searchable & config) override;
    bool close() override;

    // --------- IThreeAxisLinearAccelerometers Declarations. Implementation in IThreeAxisLinearAccelerometersImpl.cpp ---------
    size_t getNrOfThreeAxisLinearAccelerometers() const override;
    yarp::dev::MAS_status getThreeAxisLinearAccelerometerStatus(size_t sens_index) const override;
    bool getThreeAxisLinearAccelerometerName(size_t sens_index, std::string & name) const override;
    bool getThreeAxisLinearAccelerometerFrameName(size_t sens_index, std::string & frameName) const override;
    bool getThreeAxisLinearAccelerometerMeasure(size_t sens_index, yarp::sig::Vector & out, double & timestamp) const override;

    // --------- IThreeAxisGyroscopes Declarations. Implementation in IThreeAxisGyroscopesImpl.cpp ---------
    size_t getNrOfThreeAxisGyroscopes() const override;
    yarp::dev::MAS_status getThreeAxisGyroscopeStatus(size_t sens_index) const override;
    bool getThreeAxisGyroscopeName(size_t sens_index, std::string & name) const override;
    bool getThreeAxisGyroscopeFrameName(size_t sens_index, std::string & frameName) const override;
    bool getThreeAxisGyroscopeMeasure(size_t sens_index, yarp::sig::Vector & out, double & timestamp) const override;

    // --------- IThreeAxisMagnetometers Declarations. Implementation in IThreeAxisMagnetometersImpl.cpp ---------
    size_t getNrOfThreeAxisMagnetometers() const override;
    yarp::dev::MAS_status getThreeAxisMagnetometerStatus(size_t sens_index) const override;
    bool getThreeAxisMagnetometerName(size_t sens_index, std::string & name) const override;
    bool getThreeAxisMagnetometerFrameName(size_t sens_index, std::string & frameName) const override;
    bool getThreeAxisMagnetometerMeasure(size_t sens_index, yarp::sig::Vector & out, double & timestamp) const override;

private:
    // -- Helper Funcion declarations. Implementation in PhidgetSpatial.cpp --

    ///////////////////////////////////////////////////////////////////////////
    // The following six functions have been extracted and modified from the - Spatial simple -
    // example ((creates an Spatial handle, hooks the event handlers, and then waits for an
    // encoder is attached. Once it is attached, the program will wait for user input so that
    // we can see the event data on the screen when using the encoder. Legal info:
    // Copyright 2008 Phidgets Inc.  All rights reserved.
    // This work is licensed under the Creative Commons Attribution 2.5 Canada License.
    // view a copy of this license, visit http://creativecommons.org/licenses/by/2.5/ca/
    static int AttachHandler(CPhidgetHandle ENC, void * userptr);
    static int DetachHandler(CPhidgetHandle ENC, void * userptr);
    static int ErrorHandler(CPhidgetHandle ENC, void * userptr, int ErrorCode, const char * Description);
    static int SpatialDataHandler(CPhidgetSpatialHandle spatial, void * userptr, CPhidgetSpatial_SpatialEventDataHandle * data, int count);
    static int display_properties(CPhidgetSpatialHandle phid);
    ///////////////////////////////////////////////////////////////////////////

    CPhidgetSpatialHandle hSpatial0;
    mutable std::mutex mtx;

    double acceleration[3];
    double angularRate[3];
    double magneticField[3];

    double timestamp {0.0};
};

#endif // __PHIDGET_SPATIAL_HPP__
