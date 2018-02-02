// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __SPACE_NAVIGATOR__
#define __SPACE_NAVIGATOR__

#include <spnav.h>
#include <yarp/dev/IAnalogSensor.h>

#include "ColorDebug.hpp"

#define DEFAULT_NUM_CHANNELS 8

#define FULL_SCALE_X 460.0
#define FULL_SCALE_Y 430.0
#define FULL_SCALE_Z 440.0
#define FULL_SCALE_ROLL 415.0
#define FULL_SCALE_PITCH 405.0
#define FULL_SCALE_YAW 435.0

#define DEADBAND 0.125

namespace roboticslab
{

/**
 * @ingroup YarpPlugins
 * @defgroup SpaceNavigator
 * @brief Contains roboticslab::SpaceNavigator.
 */

 /**
 * @ingroup SpaceNavigator
 * @brief Implementation for the SpaceNavigator 3D mouse.
 *
 * Launch as in:
@verbatim
yarpdev --device SpaceNavigator --period 5 --name /spacenavigator
@endverbatim
 * You can split mouse and button output into separate channels with:
@verbatim
yarpdev --device SpaceNavigator --period 5 --name /spacenavigator --ports "(mouse:o buttons:o)" --channels 8 --mouse:o 0 5 0 5 --buttons:o 6 7 0 1
@endverbatim
 */
class SpaceNavigator : public yarp::dev::DeviceDriver, public yarp::dev::IAnalogSensor
{
    public:

        SpaceNavigator();

        //  --------- DeviceDriver Declarations. Implementation in DeviceDriverImpl.cpp ---------
        virtual bool open(yarp::os::Searchable& config);
        virtual bool close();

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

    protected:

        //! @brief Enforce that a value is between -1 and 1
        double enforceRange(double in);

        //! @brief Enforce the deadband (setting values within deadband to zero)
        double enforceDeadband(double in);

    private:

        double dx, dy, dz;
        double droll, dpitch, dyaw;
        int button1, button2;
        unsigned int noDataCounter;
        static const unsigned int MAX_NO_DATA_ITERATIONS;
};

}  // namespace roboticslab

#endif  // __SPACE_NAVIGATOR__
