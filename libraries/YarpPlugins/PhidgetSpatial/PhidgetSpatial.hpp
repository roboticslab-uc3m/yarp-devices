// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __PHIDGET_SPATIAL_HPP__
#define __PHIDGET_SPATIAL_HPP__

#include <mutex>

#include <yarp/conf/version.h>

#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/MultipleAnalogSensorsInterfaces.h>

#include <phidget21.h>

constexpr auto NUM_SENSORS = 1;

#if YARP_VERSION_COMPARE(>=, 4, 1, 0) || defined(YARP_NEXT)
#define CHECK_SENSOR(n) do { if ((n) < 0 || (n) > NUM_SENSORS - 1) return yarp::dev::ReturnValue_error_input_out_of_bounds; } while (0)
#else
#define CHECK_SENSOR(n) do { if ((n) < 0 || (n) > NUM_SENSORS - 1) return false; } while (0)
#endif

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

    // --------- IThreeAxisLinearAccelerometers declarations. Implementation in IThreeAxisLinearAccelerometersImpl.cpp ---------
#if YARP_VERSION_COMPARE(>=, 4, 1, 0) || defined(YARP_NEXT)
    yarp::dev::ReturnValue getNrOfThreeAxisLinearAccelerometers(std::size_t & num) const override;
    yarp::dev::MAS_status getThreeAxisLinearAccelerometerStatus(std::size_t sens_index) const override;
    yarp::dev::ReturnValue getThreeAxisLinearAccelerometerName(std::size_t sens_index, std::string & name) const override;
    yarp::dev::ReturnValue getThreeAxisLinearAccelerometerFrameName(std::size_t sens_index, std::string & frameName) const override;
    yarp::dev::ReturnValue getThreeAxisLinearAccelerometerMeasure(std::size_t sens_index, yarp::sig::Vector & out, double & timestamp) const override;
#else
    std::size_t getNrOfThreeAxisLinearAccelerometers() const override;
    yarp::dev::MAS_status getThreeAxisLinearAccelerometerStatus(std::size_t sens_index) const override;
    bool getThreeAxisLinearAccelerometerName(std::size_t sens_index, std::string & name) const override;
    bool getThreeAxisLinearAccelerometerFrameName(std::size_t sens_index, std::string & frameName) const override;
    bool getThreeAxisLinearAccelerometerMeasure(std::size_t sens_index, yarp::sig::Vector & out, double & timestamp) const override;
#endif

    // --------- IThreeAxisGyroscopes declarations. Implementation in IThreeAxisGyroscopesImpl.cpp ---------
#if YARP_VERSION_COMPARE(>=, 4, 1, 0) || defined(YARP_NEXT)
    yarp::dev::ReturnValue getNrOfThreeAxisGyroscopes(std::size_t & num) const override;
    yarp::dev::MAS_status getThreeAxisGyroscopeStatus(std::size_t sens_index) const override;
    yarp::dev::ReturnValue getThreeAxisGyroscopeName(std::size_t sens_index, std::string & name) const override;
    yarp::dev::ReturnValue getThreeAxisGyroscopeFrameName(std::size_t sens_index, std::string & frameName) const override;
    yarp::dev::ReturnValue getThreeAxisGyroscopeMeasure(std::size_t sens_index, yarp::sig::Vector & out, double & timestamp) const override;
#else
    std::size_t getNrOfThreeAxisGyroscopes() const override;
    yarp::dev::MAS_status getThreeAxisGyroscopeStatus(std::size_t sens_index) const override;
    bool getThreeAxisGyroscopeName(std::size_t sens_index, std::string & name) const override;
    bool getThreeAxisGyroscopeFrameName(std::size_t sens_index, std::string & frameName) const override;
    bool getThreeAxisGyroscopeMeasure(std::size_t sens_index, yarp::sig::Vector & out, double & timestamp) const override;
#endif

    // --------- IThreeAxisMagnetometers declarations. Implementation in IThreeAxisMagnetometersImpl.cpp ---------
#if YARP_VERSION_COMPARE(>=, 4, 1, 0) || defined(YARP_NEXT)
    yarp::dev::ReturnValue getNrOfThreeAxisMagnetometers(std::size_t & num) const override;
    yarp::dev::MAS_status getThreeAxisMagnetometerStatus(std::size_t sens_index) const override;
    yarp::dev::ReturnValue getThreeAxisMagnetometerName(std::size_t sens_index, std::string & name) const override;
    yarp::dev::ReturnValue getThreeAxisMagnetometerFrameName(std::size_t sens_index, std::string & frameName) const override;
    yarp::dev::ReturnValue getThreeAxisMagnetometerMeasure(std::size_t sens_index, yarp::sig::Vector & out, double & timestamp) const override;
#else
    std::size_t getNrOfThreeAxisMagnetometers() const override;
    yarp::dev::MAS_status getThreeAxisMagnetometerStatus(std::size_t sens_index) const override;
    bool getThreeAxisMagnetometerName(std::size_t sens_index, std::string & name) const override;
    bool getThreeAxisMagnetometerFrameName(std::size_t sens_index, std::string & frameName) const override;
    bool getThreeAxisMagnetometerMeasure(std::size_t sens_index, yarp::sig::Vector & out, double & timestamp) const override;
#endif

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
