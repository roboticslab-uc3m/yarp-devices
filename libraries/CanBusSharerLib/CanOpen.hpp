// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __CAN_OPEN_HPP__
#define __CAN_OPEN_HPP__

#include "SdoClient.hpp"

namespace roboticslab
{

class CanOpen
{
public:
    CanOpen(int id);
    ~CanOpen();

    SdoClient * sdo();
    bool configureSdo(double timeout);

private:
    int id;
    SdoClient * _sdo;
};

}  // namespace roboticslab

#endif // __CAN_OPEN_HPP__
