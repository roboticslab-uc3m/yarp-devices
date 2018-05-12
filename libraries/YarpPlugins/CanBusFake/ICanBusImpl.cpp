// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusFake.hpp"

#include <cstring>

// -----------------------------------------------------------------------------

bool roboticslab::CanBusFake::canSetBaudRate(unsigned int rate)
{
    CD_DEBUG("(%d)\n", rate);
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::CanBusFake::canGetBaudRate(unsigned int * rate)
{
    CD_DEBUG("\n");
    return 0;
}

// -----------------------------------------------------------------------------

bool roboticslab::CanBusFake::canIdAdd(unsigned int id)
{
    CD_DEBUG("(%d)\n", id);
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::CanBusFake::canIdDelete(unsigned int id)
{
    CD_DEBUG("(%d)\n", id);
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::CanBusFake::canRead(yarp::dev::CanBuffer & msgs, unsigned int size, unsigned int * read, bool wait)
{
    for (unsigned int i = 0; i < size; i++)
    {
        yarp::dev::CanMessage & msg = msgs[i];
        msg.setId(0);
        msg.setLen(sizeof(FAKE_DATA));
        unsigned char * data = msg.getData();
        data = const_cast<unsigned char *>(FAKE_DATA);
    }

    *read = size;
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::CanBusFake::canWrite(const yarp::dev::CanBuffer & msgs, unsigned int size, unsigned int * sent, bool wait)
{
    *sent = size;
    return true;
}

// -----------------------------------------------------------------------------
