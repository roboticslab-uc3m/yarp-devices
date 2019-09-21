// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __LACQUEY_FETCH_HPP__
#define __LACQUEY_FETCH_HPP__

#include <cstddef>
#include <cstdint>

#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/IControlMode.h>
#include <yarp/dev/IPWMControl.h>

#include "ICanBusSharer.hpp"

#define CHECK_JOINT(j) do { int n; if (getNumberOfMotorsRaw(&n), (j) != n - 1) return false; } while (0)

namespace roboticslab
{

/**
 * @ingroup YarpPlugins
 * \defgroup LacqueyFetch
 * @brief Contains roboticslab::LacqueyFetch.
 */

/**
 * @ingroup LacqueyFetch
 * @brief Implementation for the Lacquey Fetch hand custom UC3M circuit as a single
 * CAN bus joint (controlboard raw interfaces).
 */
class LacqueyFetch : public yarp::dev::DeviceDriver,
                     public yarp::dev::IControlModeRaw,
                     public yarp::dev::IPWMControlRaw,
                     public ICanBusSharer
{
public:

    LacqueyFetch() : canId(0), refDutyCycles(0.0), sender(nullptr)
    { }

    //  --------- DeviceDriver Declarations. Implementation in LacqueyFetch.cpp ---------
    virtual bool open(yarp::os::Searchable & config);
    virtual bool close();

    //  --------- ICanBusSharer Declarations. Implementation in LacqueyFetch.cpp ---------
    virtual unsigned int getId();
    virtual bool interpretMessage(const yarp::dev::CanMessage & message);
    virtual bool start();
    virtual bool readyToSwitchOn();
    virtual bool switchOn();
    virtual bool enable();
    virtual bool recoverFromError();
    virtual bool registerSender(CanSenderDelegate * sender);

    //  --------- IControlModeRaw Declarations. Implementation in IControlModeRawImpl.cpp ---------
    virtual bool getControlModeRaw(int j, int * mode);
    virtual bool getControlModesRaw(int * modes);
    virtual bool getControlModesRaw(const int n_joint, const int * joints, int * modes);
    virtual bool setControlModeRaw(const int j, const int mode);
    virtual bool setControlModesRaw(const int n_joint, const int *joints, int * modes);
    virtual bool setControlModesRaw(int * modes);

    // ------- IPWMControlRaw declarations. Implementation in IPWMControlRawImpl.cpp -------
    virtual bool getNumberOfMotorsRaw(int * number);
    virtual bool setRefDutyCycleRaw(int m, double ref);
    virtual bool setRefDutyCyclesRaw(const double * refs);
    virtual bool getRefDutyCycleRaw(int m, double * ref);
    virtual bool getRefDutyCyclesRaw(double * refs);
    virtual bool getDutyCycleRaw(int m, double * val);
    virtual bool getDutyCyclesRaw(double * vals);

private:

    bool send(std::size_t len, const std::uint8_t * msgData)
    { return sender->prepareMessage(message_builder(canId, len, msgData)); }

    unsigned int canId;
    double refDutyCycles;
    CanSenderDelegate * sender;
};

} // namespace roboticslab

#endif // __LACQUEY_FETCH_HPP__
