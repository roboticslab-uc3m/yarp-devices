// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __LACQUEY_FETCH_HPP__
#define __LACQUEY_FETCH_HPP__

#include <cstdint>

#include <string>

#include <yarp/conf/numeric.h>

#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/IAxisInfo.h>
#include <yarp/dev/IControlMode.h>
#include <yarp/dev/IPWMControl.h>

#include "ICanBusSharer.hpp"
#include "LacqueyFetch_ParamsParser.h"

#define CHECK_JOINT(j) do { int n; if (getNumberOfMotorsRaw(&n), (j) != n - 1) return false; } while (0)

/**
 * @ingroup YarpPlugins
 * @defgroup LacqueyFetch
 * @brief Contains LacqueyFetch.
 */

/**
 * @ingroup LacqueyFetch
 * @brief Implementation for the Lacquey Fetch hand custom UC3M circuit as a single
 * CAN bus joint (control board raw interfaces).
 */
class LacqueyFetch : public yarp::dev::DeviceDriver,
                     public yarp::dev::IAxisInfoRaw,
                     public yarp::dev::IControlModeRaw,
                     public yarp::dev::IPWMControlRaw,
                     public roboticslab::ICanBusSharer,
                     public LacqueyFetch_ParamsParser
{
public:
    //  --------- DeviceDriver declarations. Implementation in LacqueyFetch.cpp ---------

    bool open(yarp::os::Searchable & config) override;
    bool close() override;

    //  --------- ICanBusSharer declarations. Implementation in LacqueyFetch.cpp ---------

    unsigned int getId() override;
    bool notifyMessage(const roboticslab::can_message & message) override;
    bool initialize() override;
    bool finalize() override;
    bool registerSender(roboticslab::ICanSenderDelegate * sender) override;
    bool synchronize(double timestamp) override;

    //  --------- IAxisInfoRaw declarations. Implementation in IAxisInfoRawImpl.cpp ---------

    bool getAxes(int * ax) override;
    bool getAxisNameRaw(int j, std::string & name) override;
    bool getJointTypeRaw(int j, yarp::dev::JointTypeEnum & type) override;

    //  --------- IControlModeRaw declarations. Implementation in IControlModeRawImpl.cpp ---------

    bool getControlModeRaw(int j, int * mode) override;
    bool getControlModesRaw(int * modes) override;
    bool getControlModesRaw(int n_joint, const int * joints, int * modes) override;
    bool setControlModeRaw(int j, int mode) override;
    bool setControlModesRaw(int * modes) override;
    bool setControlModesRaw(int n_joint, const int * joints, int * modes) override;

    // ------- IPWMControlRaw declarations. Implementation in IPWMControlRawImpl.cpp -------

    bool getNumberOfMotorsRaw(int * number) override;
    bool setRefDutyCycleRaw(int m, double ref) override;
    bool setRefDutyCyclesRaw(const double * refs) override;
    bool getRefDutyCycleRaw(int m, double * ref) override;
    bool getRefDutyCyclesRaw(double * refs) override;
    bool getDutyCycleRaw(int m, double * val) override;
    bool getDutyCyclesRaw(double * vals) override;

private:
    static constexpr unsigned int CAN_OP = 0x780; // keep in sync with firmware

    bool send(unsigned int len, const std::uint8_t * msgData)
    { return sender && sender->prepareMessage({CAN_OP + m_canId, len, msgData}); }

    yarp::conf::float32_t refDutyCycles {0};
    roboticslab::ICanSenderDelegate * sender {nullptr};
};

#endif // __LACQUEY_FETCH_HPP__
