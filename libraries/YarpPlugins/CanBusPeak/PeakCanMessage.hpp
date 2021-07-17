// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __PEAK_CAN_MESSAGE__
#define __PEAK_CAN_MESSAGE__

#include <yarp/dev/CanBusInterface.h>

// upstream bug in the Peak API header, v8.5.1
#include <sys/time.h>

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
    ~PeakCanMessage() override;
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
    struct pcanfd_msg * message;
};

} // namespace roboticslab

#endif // __PEAK_CAN_MESSAGE__
