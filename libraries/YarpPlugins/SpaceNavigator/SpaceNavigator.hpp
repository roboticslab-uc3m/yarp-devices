// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __SPACE_NAVIGATOR_HPP__
#define __SPACE_NAVIGATOR_HPP__

#include <mutex>

#include <yarp/os/Thread.h>

#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/IJoypadController.h>

#include <spnav.h>

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
                       public yarp::dev::IJoypadController,
                       public yarp::os::Thread, // not using PeriodicThread for performance reasons
                       public SpaceNavigator_ParamsParser
{
public:
    // --------- DeviceDriver Declarations. Implementation in DeviceDriverImpl.cpp ---------
    bool open(yarp::os::Searchable & config) override;
    bool close() override;

    // --------- Thread Declarations. Implementation in ThreadImpl.cpp ---------
    void run() override;

    // --------- IAnalogSensor Declarations. Implementation in IJoypadControllerImpl.cpp ---------
    bool getAxisCount(unsigned int & axis_count) override;
    bool getButtonCount(unsigned int & button_count) override;
    bool getTrackballCount(unsigned int & trackball_count) override;
    bool getHatCount(unsigned int & hat_count) override;
    bool getTouchSurfaceCount(unsigned int & touch_count) override;
    bool getStickCount(unsigned int & stick_count) override;
    bool getStickDoF(unsigned int stick_id, unsigned int & DoF) override;
    bool getButton(unsigned int button_id, float & value) override;
    bool getTrackball(unsigned int trackball_id, yarp::sig::Vector & value) override;
    bool getHat(unsigned int hat_id, unsigned char & value) override;
    bool getAxis(unsigned int axis_id, double & value) override;
    bool getStick(unsigned int stick_id, yarp::sig::Vector & value, yarp::dev::IJoypadController::JoypadCtrl_coordinateMode coordinate_mode) override;
    bool getTouch(unsigned int touch_id, yarp::sig::Vector & value) override;

private:
    double dx {0.0};
    double dy {0.0};
    double dz {0.0};

    double drx {0.0};
    double dry {0.0};
    double drz {0.0};

    float button1 {0.f};
    float button2 {0.f};

    double deadband {0.0};

    mutable std::mutex mtx;
};

#endif // __SPACE_NAVIGATOR_HPP__
