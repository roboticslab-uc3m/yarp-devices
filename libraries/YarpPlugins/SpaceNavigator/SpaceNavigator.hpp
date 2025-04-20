// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __SPACE_NAVIGATOR_HPP__
#define __SPACE_NAVIGATOR_HPP__

#include <spnav.h>

#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/IAnalogSensor.h>

#include "SpaceNavigator_ParamsParser.h"

/**
 * @ingroup YarpPlugins
 * @defgroup SpaceNavigator
 * @brief Contains SpaceNavigator.
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
class SpaceNavigator : public yarp::dev::DeviceDriver,
                       public yarp::dev::IAnalogSensor,
                       public SpaceNavigator_ParamsParser
{
public:
    //  --------- DeviceDriver Declarations. Implementation in DeviceDriverImpl.cpp ---------
    bool open(yarp::os::Searchable & config) override;
    bool close() override;

    //  --------- IAnalogSensor Declarations. Implementation in IAnalogSensorImpl.cpp ---------

    int read(yarp::sig::Vector & out) override;
    int getState(int ch) override;
    int getChannels() override;
    int calibrateSensor() override;
    int calibrateSensor(const yarp::sig::Vector & value) override;
    int calibrateChannel(int ch) override;
    int calibrateChannel(int ch, double value) override;

private:
    double dx {0.0};
    double dy {0.0};
    double dz {0.0};

    double droll {0.0};
    double dpitch {0.0};
    double dyaw {0.0};

    int button1 {0};
    int button2 {0};

    unsigned int noDataCounter {0};
    double deadband {0.0};
};

#endif // __SPACE_NAVIGATOR_HPP__
