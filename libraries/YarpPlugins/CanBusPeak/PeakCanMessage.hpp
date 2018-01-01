// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __PEAK_CAN_MESSAGE__
#define __PEAK_CAN_MESSAGE__

#include <yarp/dev/CanBusInterface.h>

#include <pcanfd.h>

namespace roboticslab
{

/**
 * @ingroup CanBusPeak
 * @brief YARP wrapper for PeakCAN messages.
 */
class PeakCanMessage : public yarp::dev::CanMessage
{
public:
    PeakCanMessage();
    virtual ~PeakCanMessage();
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
    struct pcanfd_msg * message;
};

}  // namespace roboticslab

#endif  // __PEAK_CAN_MESSAGE__
