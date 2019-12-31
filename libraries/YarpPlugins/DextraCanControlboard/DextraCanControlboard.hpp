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
 * @defgroup DextraCanControlboard
 * @brief Contains roboticslab::DextraCanControlboard.
 */

/**
 * @ingroup DextraCanControlboard
 * @brief Synapse interface for a CAN network.
 */
class CanSynapse : public Synapse
{
public:
    //! Constructor.
    CanSynapse(unsigned int canId);

    virtual void configure(void * handle) override;

protected:
    virtual bool getMessage(unsigned char * msg, char stopByte, int size) override;
    virtual bool sendMessage(unsigned char * msg, int size) override;

private:
    unsigned int canId;
    CanSenderDelegate * sender;
};

/**
 * @ingroup DextraCanControlboard
 * @brief Implementation of a CAN-based raw controlboard for a Dextra hand.
 */
class DextraCanControlboard : public yarp::dev::DeviceDriver,
                              public DextraRawControlboard,
                              public ICanBusSharer
{
public:

    DextraCanControlboard() : canId(0)
    { }

    //  --------- DeviceDriver declarations. Implementation in DeviceDriverImpl.cpp ---------

    virtual bool open(yarp::os::Searchable & config) override;
    virtual bool close() override;

    //  --------- ICanBusSharer declarations. Implementation in ICanBusSharerImpl.cpp ---------

    virtual unsigned int getId() override;
    virtual bool initialize() override;
    virtual bool finalize() override;
    virtual bool interpretMessage(const yarp::dev::CanMessage & message) override;
    virtual bool registerSender(CanSenderDelegate * sender) override;

protected:

    unsigned int canId;
    DextraRawControlboard raw;
};

} // namespace roboticslab

#endif // __DEXTRA_CAN_CONTROLBOARD_HPP__
