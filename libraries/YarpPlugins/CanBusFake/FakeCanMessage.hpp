// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __FAKE_CAN_MESSAGE__
#define __FAKE_CAN_MESSAGE__

#include <yarp/dev/CanBusInterface.h>

namespace roboticslab
{

/**
 * @ingroup CanBusFake
 * @brief Simple structure resembling real CAN messages.
 */
 struct fake_can_msg
 {
    int id;
    unsigned char dlc;
    unsigned char * data;
 };

/**
 * @ingroup CanBusFake
 * @brief YARP wrapper for fake CAN messages.
 */
class FakeCanMessage : public yarp::dev::CanMessage
{
public:
    FakeCanMessage();
    virtual ~FakeCanMessage();
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
    struct fake_can_msg * message;
};

}  // namespace roboticslab

#endif  // __FAKE_CAN_MESSAGE__
