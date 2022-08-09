// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __FUTURE_OBSERVER_HPP__
#define __FUTURE_OBSERVER_HPP__

#include "gtest/gtest.h"

#include <chrono>
#include <functional>
#include <future>
#include <thread>
#include <utility>
#include <vector>

namespace roboticslab::test
{

/**
 * @ingroup yarp_devices_tests
 * @defgroup FutureObserverLib
 * @brief Helper stuff for async operations.
 */

/**
 * @ingroup FutureObserverLib
 * @brief Registers asynchronous operations.
 */
class FutureObserver
{
public:
    //! Virtual destructor.
    virtual ~FutureObserver() = default;

    //! Finalize all pending tasks and clean queue.
    void shutdown()
    {
        for (auto f : futures)
        {
            if (f->valid())
            {
                f->wait();
            }

            delete f;
        }
    }

    //! Register an asynchronous operation that can be assigned thereafter.
    std::future<void> & f()
    {
        auto * f = new std::future<void>;
        futures.push_back(f);
        return *f;
    }

    static constexpr int MILLIS = 50;

private:
    std::vector<std::future<void> *> futures;
};

/**
 * @ingroup FutureObserverLib
 * @brief Functor wait-with-callback class.
 */
class observer_timer final
{
public:
    //! Register function object and configure wait time.
    template<typename Fn>
    observer_timer(int _milliseconds, Fn && _fn)
        : milliseconds(_milliseconds), fn(std::move(_fn))
    { }

    //! Wait and call stored function.
    void operator()()
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(milliseconds));
        ASSERT_TRUE(fn());
    }

private:
    int milliseconds;
    std::function<bool()> fn;
};

} // namespace roboticslab::test

#endif // __FUTURE_OBSERVER_HPP__
