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
    virtual ~HicoCanMessage();
    virtual yarp::dev::CanMessage & operator=(const yarp::dev::CanMessage & l);

    virtual unsigned int getId() const;
    virtual unsigned char getLen() const;
    virtual void setLen(unsigned char len);
    virtual void setId(unsigned int id);
    virtual const unsigned char * getData() const;
    virtual unsigned char * getData();
    virtual unsigned char * getPointer();
    virtual const unsigned char * getPointer() const;
    virtual void setBuffer(unsigned char * buf);

private:
    struct can_msg * message;
};

}  // namespace roboticslab

#endif  // __HICO_CAN_MESSAGE__
