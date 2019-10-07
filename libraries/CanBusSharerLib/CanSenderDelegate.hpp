// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __CAN_SENDER_DELEGATE_HPP__
#define __CAN_SENDER_DELEGATE_HPP__

namespace roboticslab
{

struct can_message
{
    unsigned int id;
    unsigned int len;
    const unsigned char * data;
};

/**
 * @brief
 */
class CanSenderDelegate
{
public:

    virtual ~CanSenderDelegate()
    { }

    virtual bool prepareMessage(const can_message & msg) = 0;
};

} // namespace roboticslab

#endif // __CAN_SENDER_DELEGATE_HPP__
