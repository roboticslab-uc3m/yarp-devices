// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanSenderDelegate.hpp"

#include <cstring>

using namespace roboticslab;

void message_builder::operator ()(yarp::dev::CanMessage & msg) const
{
    msg.setId(id);
    msg.setLen(len);
    std::memcpy(msg.getData(), data, len * sizeof(data));
}
