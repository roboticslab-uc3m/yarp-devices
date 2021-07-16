// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __CAN_BUS_HICO_HPP__
#define __CAN_BUS_HICO_HPP__

#include <set>
#include <map>
#include <mutex>
#include <string>
#include <utility>

#include <yarp/os/Bottle.h>

#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/CanBusInterface.h>

#include "hico_api.h"
#include "HicoCanMessage.hpp"

#define DEFAULT_PORT "/dev/can0"
#define DEFAULT_BITRATE 1000000

#define DEFAULT_RX_TIMEOUT_MS 1
#define DEFAULT_TX_TIMEOUT_MS 0  // '0' means no timeout

#define DEFAULT_BLOCKING_MODE true
#define DEFAULT_ALLOW_PERMISSIVE false

#define DELAY 0.001  // [s]

#define DEFAULT_FILTER_CONFIGURATION "disabled"

namespace roboticslab
{

/**
 * @ingroup YarpPlugins
 * @defgroup CanBusHico
 * @brief Contains roboticslab::CanBusHico.
 */

/**
 * @ingroup CanBusHico
 * @brief Specifies the HicoCan (hcanpci) behaviour and specifications.
 */
class CanBusHico : public yarp::dev::DeviceDriver,
                   public yarp::dev::ICanBus,
                   public yarp::dev::ICanBusErrors,
                   public yarp::dev::ImplementCanBufferFactory<HicoCanMessage, struct can_msg>
{
public:

    CanBusHico() : fileDescriptor(0),
                   rxTimeoutMs(DEFAULT_RX_TIMEOUT_MS),
                   txTimeoutMs(DEFAULT_TX_TIMEOUT_MS),
                   blockingMode(DEFAULT_BLOCKING_MODE),
                   allowPermissive(DEFAULT_ALLOW_PERMISSIVE),
                   filterManager(nullptr),
                   filterConfig(FilterManager::DISABLED)
    { }

    ~CanBusHico() override
    { close(); }

    //  --------- DeviceDriver declarations. Implementation in DeviceDriverImpl.cpp ---------

    /** Initialize the CAN device.
     * @param device is the device path, such as "/dev/can0".
     * @param bitrate is the bitrate, such as BITRATE_100k.
     * @return true/false on success/failure.
     */
    bool open(yarp::os::Searchable& config) override;

    /** Close the CAN device. */
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

protected:

    class FilterManager
    {
    public:
        enum filter_config { DISABLED, NO_RANGE, MASK_AND_RANGE };

        explicit FilterManager(int fileDescriptor, bool enableRanges);

        bool parseIds(const yarp::os::Bottle & b);
        bool hasId(unsigned int id) const;
        bool isValid() const;
        bool insertId(unsigned int id);
        bool eraseId(unsigned int id);
        bool clearFilters(bool clearStage = true);

        static filter_config parseFilterConfiguration(const std::string & str);

        static const int MAX_FILTERS;

    private:
        bool setMaskedFilter(unsigned int id);
        bool setRangedFilter(unsigned int lower, unsigned int upper);
        bool bulkUpdate();

        int fd;
        bool valid;
        bool enableRanges;
        std::set<unsigned int> stage, currentlyActive;
    };

    enum io_operation { READ, WRITE };

    bool waitUntilTimeout(io_operation op, bool * bufferReady);

    static void initBitrateMap();
    bool bitrateToId(unsigned int bitrate, unsigned int * id);
    bool idToBitrate(unsigned int id, unsigned int * bitrate);

    static std::map<unsigned int, unsigned int> idToBitrateMap;

    /** CAN file descriptor */
    int fileDescriptor;
    int rxTimeoutMs, txTimeoutMs;

    bool blockingMode;
    bool allowPermissive;

    mutable std::mutex canBusReady;

    std::pair<bool, unsigned int> bitrateState;

    FilterManager * filterManager;

    FilterManager::filter_config filterConfig;
};

} // namespace roboticslab

#endif // __CAN_BUS_HICO_HPP__
