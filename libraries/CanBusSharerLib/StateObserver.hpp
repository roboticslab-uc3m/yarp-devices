// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __STATE_OBSERVER_HPP__
#define __STATE_OBSERVER_HPP__

#include <cstdint>
#include <cstdlib>

#include <type_traits>

namespace roboticslab
{

class StateObserverBase
{
public:
    StateObserverBase(double timeout);
    virtual ~StateObserverBase() = 0;

    bool await(void * raw = nullptr);
    bool notify(const void * raw = nullptr, std::size_t len = 0);

private:
    class Private;
    Private * impl;
};

class StateObserver : private StateObserverBase
{
public:
    StateObserver(double timeout) : StateObserverBase(timeout)
    { }

    StateObserver(const StateObserver &) = delete;
    StateObserver & operator=(const StateObserver &) = delete;

    bool await()
    { return StateObserverBase::await(); }

    bool notify()
    { return StateObserverBase::notify(); }
};

template<typename T>
class TypedStateObserver : private StateObserverBase
{
public:
    TypedStateObserver(double timeout) : StateObserverBase(timeout)
    { }

    TypedStateObserver(const TypedStateObserver &) = delete;
    TypedStateObserver & operator=(const TypedStateObserver &) = delete;

    bool await(T * raw)
    { static_assert(std::is_fundamental<T>::value, "Fundamental type is required.");
      return StateObserverBase::await(raw); }

    bool notify(const T * raw, std::size_t len = sizeof(T))
    { static_assert(std::is_fundamental<T>::value, "Fundamental type is required.");
      return StateObserverBase::notify(raw, len); }
};

} // namespace roboticslab

#endif // __STATE_OBSERVER_HPP__
