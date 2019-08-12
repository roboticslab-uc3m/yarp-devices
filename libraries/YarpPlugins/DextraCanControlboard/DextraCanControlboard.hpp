// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __DEXTRA_CAN_CONTROLBOARD_HPP__
#define __DEXTRA_CAN_CONTROLBOARD_HPP__

#include <yarp/dev/CanBusInterface.h>
#include <yarp/dev/DeviceDriver.h>

#include "DextraRawControlboard.hpp"
#include "Synapse.hpp"
#include "ICanBusSharer.hpp"

namespace roboticslab
{

/**
 * @ingroup YarpPlugins
 * \defgroup DextraCanControlboard
 * @brief Contains roboticslab::DextraCanControlboard.
 */

/**
 * @ingroup DextraCanControlboard
 */
class CanSynapse : public Synapse
{
public:
    CanSynapse(int canId);
    virtual void configure(void * handle);

protected:
    virtual bool getMessage(unsigned char * msg, char stopByte, int size);
    virtual bool sendMessage(unsigned char * msg, int size);

private:
    int canId;
    CanSenderDelegate * sender;
};

/**
 * @ingroup DextraCanControlboard
 * @brief CAN implementation for the custom UC3M Dextra Hand controlboard interfaces.
 */
class DextraCanControlboard : public yarp::dev::DeviceDriver,
                              public DextraRawControlboard,
                              public ICanBusSharer
{
public:

    DextraCanControlboard();

    //  --------- DeviceDriver Declarations. Implementation in DeviceDriverImpl.cpp ---------
    virtual bool open(yarp::os::Searchable& config);
    virtual bool close();

    //  --------- ICanBusSharer Declarations. Implementation in ICanBusSharerImpl.cpp ---------
    virtual bool setIEncodersTimedRawExternal(yarp::dev::IEncodersTimedRaw * iEncodersTimedRaw) { return true; };
    virtual bool initialize() { return true; }
    virtual bool start() { return true; };
    virtual bool readyToSwitchOn() { return true; };
    virtual bool switchOn() { return true; };
    virtual bool enable() { return true; };
    virtual bool recoverFromError() { return true; };
    virtual bool interpretMessage(const yarp::dev::CanMessage & message);
    virtual bool registerSender(CanSenderDelegate * sender);

protected:

    DextraRawControlboard raw;
};

}  // namespace roboticslab

#endif  // __DEXTRA_CAN_CONTROLBOARD_HPP__
