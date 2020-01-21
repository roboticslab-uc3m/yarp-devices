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
 * @brief ...
 */
class SdoReplier final : public yarp::os::PortReader,
                         public CanMessageNotifier
{
public:
    SdoReplier();
    ~SdoReplier();

    virtual bool read(yarp::os::ConnectionReader & reader) override;
    virtual bool notifyMessage(const can_message & msg) override;

    void configureSender(CanSenderDelegate * sender)
    { this->sender = sender; }

private:
    CanSenderDelegate * sender;

    class Private;
    Private * priv;
};

} // namespace roboticslab

#endif // __SDO_REPLIER_HPP__
