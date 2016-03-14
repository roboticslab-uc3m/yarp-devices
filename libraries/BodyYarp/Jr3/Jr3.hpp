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
class Jr3 : public yarp::dev::DeviceDriver
{

    public:

        Jr3() {
        }

        //  --------- DeviceDriver Declarations. Implementation in Jr3.cpp ---------
        virtual bool open(yarp::os::Searchable& config);
        virtual bool close();

        //  --------- ICanBusSharer Declarations. Implementation in Jr3.cpp ---------


    protected:
        six_axis_array fm0, fm1;
        force_array fs0, fs1;
        int ret, fd;
        int i;

};

}  // namespace teo

#endif  // __JR3__

