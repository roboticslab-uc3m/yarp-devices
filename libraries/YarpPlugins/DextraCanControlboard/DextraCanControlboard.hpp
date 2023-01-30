// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __DEXTRA_CAN_CONTROLBOARD_HPP__
#define __DEXTRA_CAN_CONTROLBOARD_HPP__

#include <yarp/dev/CanBusInterface.h>

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

    void configure(void * handle) override;

protected:
    bool getMessage(unsigned char * msg, char stopByte, int size) override;
    bool sendMessage(unsigned char * msg, int size) override;

private:
    unsigned int canId;
    ICanSenderDelegate * sender;
};

/**
 * @ingroup DextraCanControlboard
 * @brief Implementation of a CAN-based raw controlboard for a Dextra hand.
 */
class DextraCanControlboard : public DextraRawControlboard,
                              public ICanBusSharer
{
public:
    DextraCanControlboard() : canId(0)
    { }

    //  --------- DeviceDriver declarations. Implementation in DeviceDriverImpl.cpp ---------
    bool open(yarp::os::Searchable & config) override;
    bool close() override;

    //  --------- ICanBusSharer declarations. Implementation in ICanBusSharerImpl.cpp ---------
    unsigned int getId() override;
    bool initialize() override;
    bool finalize() override;
    bool notifyMessage(const can_message & message) override;
    bool registerSender(ICanSenderDelegate * sender) override;
    bool synchronize(double timestamp) override;

protected:
    unsigned int canId;
    DextraRawControlboard raw;
};

} // namespace roboticslab

#endif // __DEXTRA_CAN_CONTROLBOARD_HPP__
