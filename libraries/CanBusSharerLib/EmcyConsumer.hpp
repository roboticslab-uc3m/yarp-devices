// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __EMCY_CONSUMER_HPP__
#define __EMCY_CONSUMER_HPP__

#include <cstdint>

#include <functional>
#include <string>

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
    typedef std::function<void(code_t, std::uint8_t, const std::uint8_t *)> HandlerFn;

    EmcyConsumer() : codeRegistry(new EmcyCodeRegistry)
    { }

    virtual ~EmcyConsumer()
    { delete codeRegistry; }

    void accept(const std::uint8_t * data);

    template<typename T>
    void setErrorCodeRegistry()
    { delete codeRegistry;
      codeRegistry = new T; }

    void registerHandler(HandlerFn fn)
    { callback = fn; }

protected:
    HandlerFn callback;
    EmcyCodeRegistry * codeRegistry;
};

} // namespace roboticslab

#endif // __EMCY_CONSUMER_HPP__
