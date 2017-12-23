// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __CAN_BUS_HICO__
#define __CAN_BUS_HICO__

#include <set>
#include <string>

#include <yarp/os/Semaphore.h>

#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/CanBusInterface.h>

#include "hico_api.h"
#include "HicoCanMessage.hpp"

#define DEFAULT_CAN_DEVICE "/dev/can0"
#define DEFAULT_CAN_BITRATE BITRATE_1000k

#define DELAY 0.001  // [s] Required when using same driver.

namespace roboticslab
{

/**
 *
 * @ingroup CanBusHico
 * @brief Specifies the HicoCan (hcanpci) behaviour and specifications.
 *
 */
class CanBusHico : public yarp::dev::DeviceDriver,
                   public yarp::dev::ICanBus,
                   private yarp::dev::ImplementCanBufferFactory<HicoCanMessage, struct can_msg>
{

public:

    CanBusHico() : fileDescriptor(0),
                   fcntlFlags(0)
    {}

    //  --------- DeviceDriver declarations. Implementation in DeviceDriverImpl.cpp ---------

    /** Initialize the CAN device.
     * @param device is the device path, such as "/dev/can0".
     * @param bitrate is the bitrate, such as BITRATE_100k.
     * @return true/false on success/failure.
     */
    virtual bool open(yarp::os::Searchable& config);

    /** Close the CAN device. */
    virtual bool close();

    //  --------- ICanBus declarations. Implementation in ICanBusImpl.cpp ---------

    virtual bool canSetBaudRate(unsigned int rate);

    virtual bool canGetBaudRate(unsigned int * rate);

    virtual bool canIdAdd(unsigned int id);

    virtual bool canIdDelete(unsigned int id);

    virtual bool canRead(yarp::dev::CanBuffer & msgs, unsigned int size, unsigned int * read, bool wait = false);

    virtual bool canWrite(const yarp::dev::CanBuffer & msgs, unsigned int size, unsigned int * sent, bool wait = false);

protected:

    bool setFdMode(bool requestedBlocking);
    bool setDelay();
    bool clearFilters();
    bool interpretBitrate(unsigned int rate, std::string & str);

    /** CAN file descriptor */
    int fileDescriptor;

    int fcntlFlags;

    /** Unique IDs set in active acceptance filters */
    std::set<unsigned int> filteredIds;

    yarp::os::Semaphore canBusReady;

};

}  // namespace roboticslab

#endif  // __CAN_BUS_HICO__
