// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusBroker.hpp"

#include <yarp/os/LogStream.h>
#include <yarp/os/Value.h>

#include "FutureTask.hpp"
#include "ICanBusSharer.hpp"
#include "LogComponent.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool CanBusBroker::open(yarp::os::Searchable & config)
{
    const auto * buses = config.find("buses").asList();

    if (!buses)
    {
        yCError(CBB) << "Missing key \"buses\" or not a list";
        return false;
    }

    for (int i = 0; i < buses->size(); i++)
    {
        auto bus = buses->get(i).asString();

        if (!config.check(bus))
        {
            yCError(CBB) << "Missing CAN bus key:" << bus;
            return false;
        }

        if (!config.find(bus).isList())
        {
            yCError(CBB) << "Key" << bus << "must be a list";
            return false;
        }

        const auto * nodes = config.find(bus).asList();

        std::vector<std::string> names;

        for (int j = 0; j < nodes->size(); j++)
        {
            auto node = nodes->get(j).asString();
            names.push_back(node);
        }

        auto * broker = new SingleBusBroker(bus, names);
        brokers.push_back(broker);

        if (!broker->configure(config))
        {
            yCError(CBB) << "Unable to configure broker of CAN bus device" << bus;
            return false;
        }
    }

    if (config.check("syncPeriod", "SYNC message period (s)"))
    {
        auto syncPeriod = config.find("syncPeriod").asFloat64();

        if (syncPeriod <= 0.0)
        {
            yCError(CBB) << "Invalid --syncPeriod:" << syncPeriod;
            return false;
        }

        FutureTaskFactory * taskFactory = nullptr;

        if (brokers.size() > 1)
        {
            taskFactory = new ParallelTaskFactory(brokers.size());
        }
        else if (brokers.size() == 1)
        {
            taskFactory = new SequentialTaskFactory;
        }

        if (taskFactory)
        {
            syncThread = new SyncPeriodicThread(brokers, taskFactory); // owns `taskFactory`
            syncThread->setPeriod(syncPeriod);

            if (!syncThread->openPort("/sync:o"))
            {
                yCError(CBB) << "Unable to open sync port";
                return false;
            }

            yarp::os::Value * obs;

            if (config.check("syncObserver", obs, "synchronization signal observer"))
            {
                auto * observer = *reinterpret_cast<TypedStateObserver<double> * const *>(obs->asBlob());
                syncThread->setObserver(observer);
            }
        }
    }
    else
    {
        yCWarning(CBB) << "Synchronization thread not enabled, use --syncPeriod";
    }

    return true;
}

// -----------------------------------------------------------------------------

bool CanBusBroker::close()
{
    bool ok = detachAll();

    for (auto * broker : brokers)
    {
        delete broker;
    }

    brokers.clear();
    return ok;
}

// -----------------------------------------------------------------------------
