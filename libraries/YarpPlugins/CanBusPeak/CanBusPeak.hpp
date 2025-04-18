// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __CAN_BUS_PEAK_HPP__
#define __CAN_BUS_PEAK_HPP__

#include <cstdint>

#include <mutex>
#include <set>

#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/CanBusInterface.h>

#include <libpcanfd.h>

#include "PeakCanMessage.hpp"
#include "CanBusPeak_ParamsParser.h"

/**
 * @ingroup YarpPlugins
 * @defgroup CanBusPeak
 * @brief Contains CanBusPeak.
 */

/**
 * @ingroup CanBusPeak
 * @brief Custom buffer of PeakCanMessage instances.
 */
class ImplementPeakCanBufferFactory : public yarp::dev::ImplementCanBufferFactory<roboticslab::PeakCanMessage, struct pcanfd_msg>
{
public:
    yarp::dev::CanBuffer createBuffer(int elem) override
    {
        yarp::dev::CanBuffer ret;
        struct pcanfd_msg * storage = new pcanfd_msg[elem];
        yarp::dev::CanMessage ** messages = new yarp::dev::CanMessage *[elem];
        auto * tmp = new roboticslab::PeakCanMessage[elem];

        std::memset(storage, 0, sizeof(struct pcanfd_msg) * elem);

        for (int k = 0; k < elem; k++)
        {
            messages[k] = &tmp[k];
            messages[k]->setBuffer(reinterpret_cast<unsigned char *>(&storage[k]));

            // Changes wrt default implementation:
            storage[k].type = PCANFD_TYPE_CAN20_MSG;
            storage[k].flags = PCANFD_MSG_STD;
        }

        ret.resize(messages, elem);
        return ret;
    }
};

/**
 * @ingroup CanBusPeak
 * @brief Specifies the PeakCan behaviour and specifications.
 */
class CanBusPeak : public yarp::dev::DeviceDriver,
                   public yarp::dev::ICanBus,
                   public yarp::dev::ICanBusErrors,
                   public ImplementPeakCanBufferFactory,
                   public CanBusPeak_ParamsParser
{
public:

    //  --------- DeviceDriver declarations. Implementation in DeviceDriverImpl.cpp ---------

    bool open(yarp::os::Searchable& config) override;
    bool close() override;

    //  --------- ICanBus declarations. Implementation in ICanBusImpl.cpp ---------

    bool canSetBaudRate(unsigned int rate) override;
    bool canGetBaudRate(unsigned int * rate) override;
    bool canIdAdd(unsigned int id) override;
    bool canIdDelete(unsigned int id) override;
    bool canRead(yarp::dev::CanBuffer & msgs, unsigned int size, unsigned int * read, bool wait = false) override;
    bool canWrite(const yarp::dev::CanBuffer & msgs, unsigned int size, unsigned int * sent, bool wait = false) override;

    //  --------- ICanBusErrors declarations. Implementation in ICanBusErrorsImpl.cpp ---------

    bool canGetErrors(yarp::dev::CanErrors & err) override;

private:

    enum io_operation { READ, WRITE };

    bool waitUntilTimeout(io_operation op, bool * bufferReady);

    std::uint64_t computeAcceptanceCodeAndMask();

    int fileDescriptor {0};

    mutable std::mutex canBusReady;

    std::set<unsigned int> activeFilters;
};

#endif // __CAN_BUS_PEAK_HPP__
