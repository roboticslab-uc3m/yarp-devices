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
    ~FakeCanMessage() override;
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
    struct fake_can_msg * message;
};

} // namespace roboticslab

#endif // __FAKE_CAN_MESSAGE__
