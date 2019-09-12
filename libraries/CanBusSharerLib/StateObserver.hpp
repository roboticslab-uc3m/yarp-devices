// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __STATE_OBSERVER_HPP__
#define __STATE_OBSERVER_HPP__

#include <cstdint>
#include <cstdlib>

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

    bool await(T * raw)
    { return StateObserverBase::await(raw); }

    bool notify(const T * raw, std::size_t len = sizeof(T))
    { return StateObserverBase::notify(raw, len); }
};

} // namespace roboticslab

#endif // __STATE_OBSERVER_HPP__
