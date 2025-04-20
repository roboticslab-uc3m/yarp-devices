// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __CAN_BUS_HICO_HPP__
#define __CAN_BUS_HICO_HPP__

#include <set>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include <yarp/os/Bottle.h>

#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/CanBusInterface.h>

#include "hico_api.h"
#include "HicoCanMessage.hpp"
#include "CanBusHico_ParamsParser.h"

/**
 * @ingroup YarpPlugins
 * @defgroup CanBusHico
 * @brief Contains CanBusHico.
 */

/**
 * @ingroup CanBusHico
 * @brief Specifies the HicoCan (hcanpci) behaviour and specifications.
 */
class CanBusHico : public yarp::dev::DeviceDriver,
                   public yarp::dev::ICanBus,
                   public yarp::dev::ICanBusErrors,
                   public yarp::dev::ImplementCanBufferFactory<roboticslab::HicoCanMessage, struct can_msg>,
                   public CanBusHico_ParamsParser
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

    class FilterManager
    {
    public:
        enum filter_config { DISABLED, NO_RANGE, MASK_AND_RANGE };

        explicit FilterManager(const CanBusHico & owner, int fileDescriptor, bool enableRanges);

        bool parseIds(const std::vector<int> & ids);
        bool hasId(unsigned int id) const;
        bool isValid() const;
        bool insertId(unsigned int id);
        bool eraseId(unsigned int id);
        bool clearFilters(bool clearStage = true);

        static constexpr int MAX_FILTERS = 4;

    private:
        bool setMaskedFilter(unsigned int id);
        bool setRangedFilter(unsigned int lower, unsigned int upper);
        bool bulkUpdate();

        const CanBusHico & owner;
        int fd {0};
        bool valid {false};
        bool enableRanges {false};
        std::set<unsigned int> stage, currentlyActive;
    };

    FilterManager::filter_config parseFilterConfiguration(const std::string & str);

    enum io_operation { READ, WRITE };

    bool waitUntilTimeout(io_operation op, bool * bufferReady);
    bool bitrateToId(unsigned int bitrate, unsigned int * id);
    bool idToBitrate(unsigned int id, unsigned int * bitrate);

    int fileDescriptor {0};

    mutable std::mutex canBusReady;

    std::pair<bool, unsigned int> bitrateState;

    FilterManager * filterManager {nullptr};

    FilterManager::filter_config filterConfig;
};

#endif // __CAN_BUS_HICO_HPP__
