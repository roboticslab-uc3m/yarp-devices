// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __CAN_OPEN_HPP__
#define __CAN_OPEN_HPP__

#include <unordered_map>

#include "SdoClient.hpp"
#include "PdoProtocol.hpp"

namespace roboticslab
{

class CanOpen
{
public:
    CanOpen(int id);
    ~CanOpen();

    SdoClient * sdo();
    bool configureSdo(double timeout);

    ReceivePdo * rpdo(unsigned int n);
    bool configureRpdo(unsigned int n);

    TransmitPdo * tpdo(unsigned int n);
    bool configureTpdo(unsigned int n);

private:
    int id;
    SdoClient * _sdo;
    std::unordered_map<unsigned int, ReceivePdo *> rpdos;
    std::unordered_map<unsigned int, TransmitPdo *> tpdos;
};

}  // namespace roboticslab

#endif // __CAN_OPEN_HPP__
