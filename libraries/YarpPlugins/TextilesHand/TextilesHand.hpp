// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __TEXTILES_HAND_HPP__
#define __TEXTILES_HAND_HPP__

#include <yarp/conf/version.h>

#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/IControlMode.h>
#include <yarp/dev/IPositionDirect.h>
#include <yarp/dev/ISerialDevice.h>

#include "TextilesHand_ParamsParser.h"

/**
 * @ingroup YarpPlugins
 * @defgroup TextilesHand
 * @brief Contains TextilesHand.
 */

/**
 * @ingroup TextilesHand
 * @brief Implementation for the custom UC3M Textiles Hand as a single CAN bus joint (control board raw interfaces).
 */
class TextilesHand : public yarp::dev::DeviceDriver,
                     public yarp::dev::IControlMode,
                     public yarp::dev::IPositionDirect,
                     public TextilesHand_ParamsParser
{
public:
    //  --------- DeviceDriver Declarations. Implementation in DeviceDriverImpl.cpp ---------

    bool open(yarp::os::Searchable & config) override;
    bool close() override;

    //  --------- IControlMode Declarations. Implementation in IControlModeImpl.cpp ---------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    yarp::dev::ReturnValue getAvailableControlModes(int j, std::vector<yarp::dev::SelectableControlModeEnum> & avail) override;
    yarp::dev::ReturnValue getControlMode(int j, yarp::dev::ControlModeEnum & mode) override;
    yarp::dev::ReturnValue getControlModes(std::vector<yarp::dev::ControlModeEnum> & modes) override;
    yarp::dev::ReturnValue getControlModes(const std::vector<int> & joints, std::vector<yarp::dev::ControlModeEnum> & modes) override;
    yarp::dev::ReturnValue setControlMode(int j, yarp::dev::SelectableControlModeEnum mode) override;
    yarp::dev::ReturnValue setControlModes(const std::vector<int> & joints, const std::vector<yarp::dev::SelectableControlModeEnum> & modes) override;
    yarp::dev::ReturnValue setControlModes(const std::vector<yarp::dev::SelectableControlModeEnum> & modes) override;
#else
    bool getControlMode(int j, int * mode) override;
    bool getControlModes(int * modes) override;
    bool getControlModes(int n_joint, const int * joints, int * modes) override;
    bool setControlMode(int j, const int mode) override;
    bool setControlModes(int n_joint, const int * joints, int * modes) override;
    bool setControlModes(int * modes) override;
#endif

    // ------- IPositionDirect declarations. Implementation in IPositionDirectImpl.cpp -------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    yarp::dev::ReturnValue getAxes(std::size_t & ax) override;
    yarp::dev::ReturnValue setPosition(int j, double ref) override;
    yarp::dev::ReturnValue setPositions(int n_joint, const int * joints, const double * refs) override;
    yarp::dev::ReturnValue setPositions(const double * refs) override;
    yarp::dev::ReturnValue getRefPosition(int joint, double * ref) override;
    yarp::dev::ReturnValue getRefPositions(double * refs) override;
    yarp::dev::ReturnValue getRefPositions(int n_joint, const int * joints, double * refs) override;
#else
    bool getAxes(int * ax) override;
    bool setPosition(int j, double ref) override;
    bool setPositions(const double * refs) override;
    bool setPositions(int n_joint, const int * joints, const double * refs) override;
    bool getRefPosition(int joint, double * ref) override;
    bool getRefPositions(double * refs) override;
    bool getRefPositions(int n_joint, const int * joints, double * refs) override;
#endif

private:
    double lastTarget {0.0};

    yarp::dev::PolyDriver serialDevice;
    yarp::dev::ISerialDevice * iSerialDevice {nullptr};
};

#endif // __TEXTILES_HAND_HPP__
