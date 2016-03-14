// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __JR3__
#define __JR3__

#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <sstream>

#include "jr3pci-ioctl.h"

//#define CD_FULL_FILE  //-- Can be globally managed from father CMake. Good for debugging with polymorphism.
//#define CD_HIDE_DEBUG  //-- Can be globally managed from father CMake.
//#define CD_HIDE_SUCCESS  //-- Can be globally managed from father CMake.
//#define CD_HIDE_INFO  //-- Can be globally managed from father CMake.
//#define CD_HIDE_WARNING  //-- Can be globally managed from father CMake.
//#define CD_HIDE_ERROR  //-- Can be globally managed from father CMake.
#include "ColorDebug.hpp"
#include "ICanBusSharer.h"


namespace teo
{

/**
 * @ingroup BodyYarp
 * \defgroup Jr3
 * @brief Contains teo::Jr3.
 */

 /**
 * @ingroup Jr3
 * @brief Implementation for the JR3 sensor.
 *
 */
class Jr3 : public yarp::dev::DeviceDriver, public yarp::dev::IGenericSensor
{

    public:

        Jr3() {
        }

        //  --------- DeviceDriver Declarations. Implementation in DeviceDriverImpl.cpp ---------
        virtual bool open(yarp::os::Searchable& config);
        virtual bool close();

        //  --------- IGenericSensor Declarations. Implementation in IGenericSensorImpl.cpp ---------
        /**
         * Read a vector from the sensor.
         * @param out a vector containing the sensor's last readings.
         * @return true/false success/failure
         */
        virtual bool read(yarp::sig::Vector &out);

        /**
         * Get the number of channels of the sensor.
         * @param nc pointer to storage, return value
         * @return true/false success/failure
         */
        virtual bool getChannels(int *nc);

        /**
         * Calibrate the sensor, single channel.
         * @param ch channel number
         * @param v reset valure
         * @return true/false success/failure
         */
        virtual bool calibrate(int ch, double v);

    protected:
        six_axis_array fm0, fm1;
        force_array fs0, fs1;
        int ret, fd;
        int i;

};

}  // namespace teo

#endif  // __JR3__

