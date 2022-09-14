// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __TECHNOSOFT_IPOS_EMCY_HPP__
#define __TECHNOSOFT_IPOS_EMCY_HPP__

#include "EmcyConsumer.hpp"

namespace roboticslab
{

/**
 * @ingroup TechnosoftIpos
 * @brief Custom EMCY messages.
 */
class TechnosoftIposEmcy : public EmcyCodeRegistry
{
public:
    std::string codeToMessage(std::uint16_t code) override;
};

} // namespace roboticslab

#endif // __TECHNOSOFT_IPOS_EMCY_HPP__
