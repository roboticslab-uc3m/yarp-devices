// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __HICO_CAN_MESSAGE__
#define __HICO_CAN_MESSAGE__

#include <yarp/dev/CanBusInterface.h>

#include "hico_api.h"

namespace roboticslab
{

/**
 * @ingroup CanBusHico
 * @brief YARP wrapper for HicoCAN messages.
 */
class HicoCanMessage : public yarp::dev::CanMessage
{
public:
    HicoCanMessage();
    ~HicoCanMessage() override;
    yarp::dev::CanMessage & operator=(const yarp::dev::CanMessage & l) override;

    unsigned int getId() const override;
    unsigned char getLen() const override;
    void setLen(unsigned char len) override;
    void setId(unsigned int id) override;
    const unsigned char * getData() const override;
    unsigned char * getData() override;
    unsigned char * getPointer() override;
    const unsigned char * getPointer() const override;
    void setBuffer(unsigned char * buf) override;

private:
    struct can_msg * message;
};

} // namespace roboticslab

#endif // __HICO_CAN_MESSAGE__
