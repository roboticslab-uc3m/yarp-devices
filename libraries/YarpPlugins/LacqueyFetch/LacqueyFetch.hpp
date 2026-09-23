// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __LACQUEY_FETCH_HPP__
#define __LACQUEY_FETCH_HPP__

#include <cstdint>

#include <string>

#include <yarp/conf/numeric.h>
#include <yarp/conf/version.h>

#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/IAxisInfo.h>
#include <yarp/dev/IControlMode.h>
#include <yarp/dev/IPWMControl.h>

#include "ICanBusSharer.hpp"
#include "LacqueyFetch_ParamsParser.h"

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
#define CHECK_JOINT(j) do { int n; if (getNumberOfMotorsRaw(&n), (j) != n - 1) return yarp::dev::ReturnValue_error_input_out_of_bounds; } while (0)
#else
#define CHECK_JOINT(j) do { int n; if (getNumberOfMotorsRaw(&n), (j) != n - 1) return false; } while (0)
#endif

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

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    yarp::dev::ReturnValue getAxes(std::size_t & ax) override;
    yarp::dev::ReturnValue getAxisNameRaw(int j, std::string & name) override;
    yarp::dev::ReturnValue getJointTypeRaw(int j, yarp::dev::JointTypeEnum & type) override;
#else
    bool getAxes(int * ax) override;
    bool getAxisNameRaw(int j, std::string & name) override;
    bool getJointTypeRaw(int j, yarp::dev::JointTypeEnum & type) override;
#endif

    //  --------- IControlModeRaw declarations. Implementation in IControlModeRawImpl.cpp ---------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    yarp::dev::ReturnValue getAvailableControlModesRaw(int j, std::vector<yarp::dev::SelectableControlModeEnum> & avail) override;
    yarp::dev::ReturnValue getControlModeRaw(int j, yarp::dev::ControlModeEnum & mode) override;
    yarp::dev::ReturnValue getControlModesRaw(std::vector<yarp::dev::ControlModeEnum> & modes) override;
    yarp::dev::ReturnValue getControlModesRaw(const std::vector<int> & joints, std::vector<yarp::dev::ControlModeEnum> & modes) override;
    yarp::dev::ReturnValue setControlModeRaw(int j, yarp::dev::SelectableControlModeEnum mode) override;
    yarp::dev::ReturnValue setControlModesRaw(const std::vector<int> & joints, const std::vector<yarp::dev::SelectableControlModeEnum> & modes) override;
    yarp::dev::ReturnValue setControlModesRaw(const std::vector<yarp::dev::SelectableControlModeEnum> & modes) override;
#else
    bool getControlModeRaw(int j, int * mode) override;
    bool getControlModesRaw(int * modes) override;
    bool getControlModesRaw(int n_joint, const int * joints, int * modes) override;
    bool setControlModeRaw(int j, int mode) override;
    bool setControlModesRaw(int * modes) override;
    bool setControlModesRaw(int n_joint, const int * joints, int * modes) override;
#endif

    // ------- IPWMControlRaw declarations. Implementation in IPWMControlRawImpl.cpp -------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    yarp::dev::ReturnValue getNumberOfMotorsRaw(int * number) override;
    yarp::dev::ReturnValue setRefDutyCycleRaw(int m, double ref) override;
    yarp::dev::ReturnValue setRefDutyCyclesRaw(const double * refs) override;
    yarp::dev::ReturnValue getRefDutyCycleRaw(int m, double * ref) override;
    yarp::dev::ReturnValue getRefDutyCyclesRaw(double * refs) override;
    yarp::dev::ReturnValue getDutyCycleRaw(int m, double * val) override;
    yarp::dev::ReturnValue getDutyCyclesRaw(double * vals) override;
#else
    bool getNumberOfMotorsRaw(int * number) override;
    bool setRefDutyCycleRaw(int m, double ref) override;
    bool setRefDutyCyclesRaw(const double * refs) override;
    bool getRefDutyCycleRaw(int m, double * ref) override;
    bool getRefDutyCyclesRaw(double * refs) override;
    bool getDutyCycleRaw(int m, double * val) override;
    bool getDutyCyclesRaw(double * vals) override;
#endif

private:
    static constexpr unsigned int CAN_OP = 0x780; // keep in sync with firmware

    bool send(unsigned int len, const std::uint8_t * msgData)
    { return sender && sender->prepareMessage({CAN_OP + m_canId, len, msgData}); }

    yarp::conf::float32_t refDutyCycles {0};
    roboticslab::ICanSenderDelegate * sender {nullptr};
};

#endif // __LACQUEY_FETCH_HPP__
