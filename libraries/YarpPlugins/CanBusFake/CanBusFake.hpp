// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __CAN_BUS_FAKE__
#define __CAN_BUS_FAKE__

#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <yarp/dev/CanBusInterface.h>

#include <ColorDebug.h>

#include "FakeCanMessage.hpp"

namespace roboticslab
{

/**
 * @ingroup YarpPlugins
 * @defgroup CanBusFake
 * @brief Contains roboticslab::CanBusFake.
 */

/**
 * @ingroup CanBusFake
 * @brief Fake CanBus driver, e.g. for testing CanBusControlboard with pure USB devices.
 */
class CanBusFake : public yarp::dev::DeviceDriver,
                   public yarp::dev::ICanBus,
                   public yarp::dev::ImplementCanBufferFactory<FakeCanMessage, struct fake_can_msg>
{

public:

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

private:

    static const unsigned char FAKE_DATA[8]; // defined in DeviceDriverImpl.cpp
};

}  // namespace roboticslab

#endif  // __CAN_BUS_FAKE__
