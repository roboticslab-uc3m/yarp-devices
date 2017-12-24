// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __CAN_BUS_PEAK__
#define __CAN_BUS_PEAK__

#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/CanBusInterface.h>

#include "PeakCanMessage.hpp"

namespace roboticslab
{

/**
 *
 * @ingroup CanBusPeak
 * @brief Specifies the PeakCan behaviour and specifications.
 *
 */
class CanBusPeak : public yarp::dev::DeviceDriver,
                   public yarp::dev::ICanBus/*,
                   private yarp::dev::ImplementCanBufferFactory<PeakCanMessage, struct can_msg>*/
{

public:

    CanBusPeak() {}

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

};

}  // namespace roboticslab

#endif  // __CAN_BUS_PEAK__
