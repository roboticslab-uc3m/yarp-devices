// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __EMCY_CONSUMER_HPP__
#define __EMCY_CONSUMER_HPP__

#include <cstdint>

#include <functional>
#include <string>

#include "SdoClient.hpp"

namespace roboticslab
{

class EmcyCodeRegistry
{
public:
    virtual ~EmcyCodeRegistry() = default;
    virtual std::string codeToMessage(std::uint16_t code);
};

class EmcyConsumer final
{
public:
    typedef std::pair<std::uint16_t, std::string> code_t; // emergency error code

    EmcyConsumer(SdoClient * sdo) : codeRegistry(new EmcyCodeRegistry), sdo(sdo)
    { }

    virtual ~EmcyConsumer()
    { delete codeRegistry; }

    bool configure(std::uint16_t inhibitTime);

    bool accept(const std::uint8_t * data);

    template<typename T>
    void setErrorCodeRegistry()
    { delete codeRegistry;
      codeRegistry = new T; }

    template<typename Fn>
    void registerHandler(Fn && fn)
    { callback = fn; }

    void unregisterHandler()
    { callback = HandlerFn(); }

private:
    typedef std::function<void(code_t, std::uint8_t, const std::uint8_t *)> HandlerFn;

    HandlerFn callback;
    EmcyCodeRegistry * codeRegistry;
    SdoClient * sdo;
};

} // namespace roboticslab

#endif // __EMCY_CONSUMER_HPP__
