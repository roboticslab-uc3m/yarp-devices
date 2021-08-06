// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __SDO_REPLIER_HPP__
#define __SDO_REPLIER_HPP__

#include <yarp/os/PortReader.h>

#include "CanMessageNotifier.hpp"
#include "CanSenderDelegate.hpp"

namespace roboticslab
{

/**
 * @ingroup CanBusControlboard
 * @brief RPC replier for remote SDO transfers.
 *
 * This class provides a callback that allows clients to interface with a remote
 * CAN network via SDO commands. Every transfer (either upload-from-drive request
 * or download-to-drive indication) is confirmed and its response sent back to
 * the RPC client.
 */
class SdoReplier final : public yarp::os::PortReader,
                         public CanMessageNotifier
{
public:
    //! Constructor.
    SdoReplier();

    //! Destructor.
    ~SdoReplier() override;

    //! Read this object from the network.
    bool read(yarp::os::ConnectionReader & reader) override;

    //! Tell observers a new CAN message has arrived.
    bool notifyMessage(const can_message & msg) override;

    //! Configure CAN sender handle.
    void configureSender(CanSenderDelegate * sender)
    { this->sender = sender; }

private:
    CanSenderDelegate * sender;

    class Private;
    Private * priv;
};

} // namespace roboticslab

#endif // __SDO_REPLIER_HPP__
