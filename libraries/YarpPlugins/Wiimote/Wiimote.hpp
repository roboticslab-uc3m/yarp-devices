// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __WIIMOTE_HPP__
#define __WIIMOTE_HPP__

#include <poll.h>

#include <mutex>

#include <yarp/conf/version.h>

#include <yarp/os/Thread.h>

#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/IJoypadController.h>

#include <xwiimote.h>

#include "Wiimote_ParamsParser.h"

/**
 * @ingroup YarpPlugins
 * @defgroup Wiimote
 * @brief Contains Wiimote.
 */

/**
 * @ingroup Wiimote
 * @brief Holds key state data from input events.
 */
struct WiimoteEventData
{
    int accelX {0};
    int accelY {0};
    int accelZ {0};

    bool buttonA {false};
    bool buttonB {false};

    bool button1 {false};
    bool button2 {false};
};

/**
 * @ingroup Wiimote
 * @brief Thread that listens to Wiimote events.
 */
class WiimoteDispatcherThread : public yarp::os::Thread
{
public:

    //! @brief Called just before a new thread starts.
    void beforeStart() override;

    //! @brief Main body of the new thread.
    void run() override;

    //! @brief Set pointer to @ref xwii_iface.
    void setInterfacePointer(struct xwii_iface * iface)
    {
        this->iface = iface;
    }

    //! @brief Retrieve event data object.
    WiimoteEventData getEventData() const;

private:

    struct xwii_iface * iface {nullptr};
    struct xwii_event event;

    WiimoteEventData eventData;
    mutable std::mutex eventDataMutex;

    struct pollfd fds[2];
    int fds_num {0};
};

/**
 * @ingroup Wiimote
 * @brief Implementation for the Wiimote controller.
 */
class Wiimote : public yarp::dev::DeviceDriver,
                public yarp::dev::IJoypadController,
                public Wiimote_ParamsParser
{
public:
    // --------- DeviceDriver Declarations. Implementation in DeviceDriverImpl.cpp ---------
    bool open(yarp::os::Searchable& config) override;
    bool close() override;

    // --------- IJoypadController Declarations. Implementation in IJoypadControllerImpl.cpp ---------
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    yarp::dev::ReturnValue getAxisCount(std::size_t & axis_count) override;
    yarp::dev::ReturnValue getButtonCount(std::size_t & button_count) override;
    yarp::dev::ReturnValue getTrackballCount(std::size_t & trackball_count) override;
    yarp::dev::ReturnValue getHatCount(std::size_t & hat_count) override;
    yarp::dev::ReturnValue getTouchSurfaceCount(std::size_t & touch_count) override;
    yarp::dev::ReturnValue getStickCount(std::size_t & stick_count) override;
    yarp::dev::ReturnValue getButton(std::size_t button_id, double & value) override;
    yarp::dev::ReturnValue getTrackball(std::size_t trackball_id, yarp::dev::TrackballData & value) override;
    yarp::dev::ReturnValue getHat(std::size_t hat_id, unsigned char & value) override;
    yarp::dev::ReturnValue getAxis(std::size_t axis_id, double & value) override;
    yarp::dev::ReturnValue getAllAxes(std::vector<double> & values) override;
    yarp::dev::ReturnValue getStick(std::size_t stick_id, yarp::dev::StickData & value, yarp::dev::IJoypadController::JoypadCtrl_coordinateMode coordinate_mode) override;
    yarp::dev::ReturnValue getTouch(std::size_t touch_id, std::vector<yarp::dev::TouchData> & value) override;
#else
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
#endif

private:
    static char * getDevicePath(int id);

    struct xwii_iface * iface {nullptr};

    WiimoteDispatcherThread dispatcherThread;
};

#endif // __WIIMOTE_HPP__
