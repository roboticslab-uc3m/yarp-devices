// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __LACQUEY_FETCH_HPP__
#define __LACQUEY_FETCH_HPP__

#include <cstdint>

#include <yarp/conf/numeric.h>

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

    LacqueyFetch() : canId(0), refDutyCycles(), sender(nullptr)
    { }

    //  --------- DeviceDriver declarations. Implementation in LacqueyFetch.cpp ---------

    virtual bool open(yarp::os::Searchable & config) override;
    virtual bool close() override;

    //  --------- ICanBusSharer declarations. Implementation in LacqueyFetch.cpp ---------

    virtual unsigned int getId() override;
    virtual bool interpretMessage(const yarp::dev::CanMessage & message) override;
    virtual bool initialize() override;
    virtual bool finalize() override;
    virtual bool registerSender(CanSenderDelegate * sender) override;

    //  --------- IControlModeRaw declarations. Implementation in IControlModeRawImpl.cpp ---------

    virtual bool getControlModeRaw(int j, int * mode) override;
    virtual bool getControlModesRaw(int * modes) override;
    virtual bool getControlModesRaw(int n_joint, const int * joints, int * modes) override;
    virtual bool setControlModeRaw(int j, int mode) override;
    virtual bool setControlModesRaw(int * modes) override;
    virtual bool setControlModesRaw(int n_joint, const int * joints, int * modes) override;

    // ------- IPWMControlRaw declarations. Implementation in IPWMControlRawImpl.cpp -------

    virtual bool getNumberOfMotorsRaw(int * number) override;
    virtual bool setRefDutyCycleRaw(int m, double ref) override;
    virtual bool setRefDutyCyclesRaw(const double * refs) override;
    virtual bool getRefDutyCycleRaw(int m, double * ref) override;
    virtual bool getRefDutyCyclesRaw(double * refs) override;
    virtual bool getDutyCycleRaw(int m, double * val) override;
    virtual bool getDutyCyclesRaw(double * vals) override;

private:

    bool send(unsigned int len, const std::uint8_t * msgData)
    { return sender->prepareMessage({canId, len, msgData}); }

    unsigned int canId;
    yarp::conf::float32_t refDutyCycles;
    CanSenderDelegate * sender;
};

} // namespace roboticslab

#endif // __LACQUEY_FETCH_HPP__
