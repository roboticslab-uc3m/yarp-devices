// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __EMCY_CONSUMER_HPP__
#define __EMCY_CONSUMER_HPP__

#include <cstdint>

#include <functional>
#include <string>

namespace roboticslab
{

/**
 * @ingroup CanOpenNodeLib
 * @brief Generic EMCY message parser.
 */
class EmcyCodeRegistry
{
public:
    //! Virtual destructor.
    virtual ~EmcyCodeRegistry() = default;

    //! Obtain string representation of an EMCY code.
    virtual std::string codeToMessage(std::uint16_t code);
};

/**
 * @ingroup CanOpenNodeLib
 * @brief Representation of CAN EMCY protocol.
 */
class EmcyConsumer final
{
public:
    typedef std::pair<std::uint16_t, std::string> code_t; ///< Emergency error code

    //! Constructor.
    EmcyConsumer() : codeRegistry(new EmcyCodeRegistry)
    { }

    //! Destructor.
    ~EmcyConsumer()
    { delete codeRegistry; }

    //! Invoke callback on parsed CAN message data.
    bool accept(const std::uint8_t * data);

    //! Instantiate a non-default EMCY message parser.
    template<typename T>
    void setErrorCodeRegistry()
    { delete codeRegistry;
      codeRegistry = new T; }

    //! Register callback.
    template<typename Fn>
    void registerHandler(Fn && fn)
    { callback = fn; }

    //! Unregister callback.
    void unregisterHandler()
    { callback = HandlerFn(); }

private:
    typedef std::function<void(code_t, std::uint8_t, const std::uint8_t *)> HandlerFn;

    HandlerFn callback;
    EmcyCodeRegistry * codeRegistry;
};

} // namespace roboticslab

#endif // __EMCY_CONSUMER_HPP__
