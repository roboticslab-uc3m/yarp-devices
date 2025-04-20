// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __TEXTILES_HAND_HPP__
#define __TEXTILES_HAND_HPP__

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

    bool getControlMode(int j, int * mode) override;
    bool getControlModes(int * modes) override;
    bool getControlModes(int n_joint, const int * joints, int * modes) override;
    bool setControlMode(int j, int mode) override;
    bool setControlModes(int * modes) override;
    bool setControlModes(int n_joint, const int * joints, int * modes) override;

    // ------- IPositionDirect declarations. Implementation in IPositionDirectImpl.cpp -------

    bool getAxes(int * ax) override;
    bool setPosition(int j, double ref) override;
    bool setPositions(const double * refs) override;
    bool setPositions(int n_joint, const int * joints, const double * refs) override;
    bool getRefPosition(int joint, double * ref) override;
    bool getRefPositions(double * refs) override;
    bool getRefPositions(int n_joint, const int * joints, double * refs) override;

private:
    double lastTarget {0.0};

    yarp::dev::PolyDriver serialDevice;
    yarp::dev::ISerialDevice * iSerialDevice {nullptr};
};

#endif // __TEXTILES_HAND_HPP__
