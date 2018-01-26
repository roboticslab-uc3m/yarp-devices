// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __PHIDGET_SPATIAL__
#define __PHIDGET_SPATIAL__

#include <yarp/os/all.h>
#include <yarp/dev/Drivers.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/IAnalogSensor.h>

#include <vector>
#include <math.h>

#include <phidget21.h>

using namespace yarp::os;
using namespace yarp::dev;

#define DEFAULT_RATE_MS 20.0
#define DEFAULT_NUM_CHANNELS 11

namespace rd
{

/**
 * @ingroup YarpPlugins
 * @brief PhidgetSpatial
 */
class PhidgetSpatial : public DeviceDriver, public yarp::dev::IAnalogSensor
{
public:

    // -------- DeviceDriver declarations. Implementation in DeviceDriverImpl.cpp --------

    /**
     * Open the DeviceDriver. 
     * @return true/false upon success/failure
     */
    virtual bool open(Searchable& config);
    virtual bool close();

    /**
     * Destructor.
     */
     ~PhidgetSpatial() {}

    //  --------- IAnalogSensor Declarations. Implementation in IAnalogSensorImpl.cpp ---------

    /**
     * Read a vector from the sensor.
     * @param out a vector containing the sensor's last readings.
     * @return AS_OK or return code. AS_TIMEOUT if the sensor timed-out.
     */
    virtual int read(yarp::sig::Vector &out);

    /**
     * Check the state value of a given channel.
     * @param ch channel number.
     * @return status.
     */
    virtual int getState(int ch);

    /**
     * Get the number of channels of the sensor.
     * @return number of channels (0 in case of errors).
     */
    virtual int getChannels();

    /**
     * Calibrates the whole sensor.
     * @return status.
     */
    virtual int calibrateSensor();

    /**
     * Calibrates the whole sensor, using an vector of calibration values.
     * @param value a vector of calibration values.
     * @return status.
     */
    virtual int calibrateSensor(const yarp::sig::Vector& value);

    /**
     * Calibrates one single channel.
     * @param ch channel number.
     * @return status.
     */
    virtual int calibrateChannel(int ch);

    /**
     * Calibrates one single channel, using a calibration value.
     * @param ch channel number.
     * @param value calibration value.
     * @return status.
     */
    virtual int calibrateChannel(int ch, double value);


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

// ------------------------------- Private -------------------------------------

private:

    CPhidgetSpatialHandle hSpatial0;
    yarp::os::Semaphore hSemaphore;
    double acceleration[3];
    double angularRate[3];
    double magneticField[3];

};

}  // namespace rd

#endif

