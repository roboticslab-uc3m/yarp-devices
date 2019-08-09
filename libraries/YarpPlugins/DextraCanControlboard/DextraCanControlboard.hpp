// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __DEXTRA_CAN_CONTROLBOARD_HPP__
#define __DEXTRA_CAN_CONTROLBOARD_HPP__

#include <mutex>

#include <yarp/dev/CanBusInterface.h>
#include <yarp/dev/DeviceDriver.h>

#include "DextraRawControlboard.hpp"
#include "Synapse.hpp"

#include "ICanBusSharer.h"

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
    CanSynapse(int canId, yarp::dev::ICanBufferFactory *iCanBufferFactory);
    ~CanSynapse();

    virtual void configure(void * handle);

protected:
    virtual bool getMessage(unsigned char * msg, char stopByte, int size);
    virtual bool sendMessage(char * msg, int size);

private:
    int canId;
    yarp::dev::ICanBus * iCanBus;
    yarp::dev::ICanBufferFactory * iCanBufferFactory;
    yarp::dev::CanBuffer canBuffer;
    mutable std::mutex mtx;
};

/**
 * @ingroup DextraCanControlboard
 * @brief CAN implementation for the custom UC3M Dextra Hand controlboard interfaces.
 */
class DextraCanControlboard : public yarp::dev::DeviceDriver,
                              public DextraRawControlboard
{
public:

    DextraCanControlboard();

    //  --------- DeviceDriver Declarations. Implementation in DeviceDriverImpl.cpp ---------
    virtual bool open(yarp::os::Searchable& config);
    virtual bool close();

    //  --------- ICanBusSharer Declarations. Implementation in ICanBusSharerImpl.cpp ---------
    virtual bool setCanBusPtr(yarp::dev::ICanBus * canDevicePtr);
    virtual bool setIEncodersTimedRawExternal(yarp::dev::IEncodersTimedRaw * iEncodersTimedRaw) { return true; };
    virtual bool initialize() { return true; }
    virtual bool start() { return true; };
    virtual bool readyToSwitchOn() { return true; };
    virtual bool switchOn() { return true; };
    virtual bool enable() { return true; };
    virtual bool recoverFromError() { return true; };
    virtual bool interpretMessage(const yarp::dev::CanMessage & message);

protected:

    DextraRawControlboard raw;
};

}  // namespace roboticslab

#endif  // __DEXTRA_CAN_CONTROLBOARD_HPP__
