// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusBroker.hpp"

#include <string>

#include <yarp/os/LogStream.h>
#include <yarp/os/Value.h>

#include "ICanBusSharer.hpp"
#include "LogComponent.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool CanBusBroker::open(yarp::os::Searchable & config)
{
    yarp::os::Property options;
    options.fromString(config.findGroup("global").toString());
    options.fromString(config.findGroup("common").toString(), false); // override global options
    options.fromString(config.toString(), false); // override common options

    if (yarp::os::Value * v; options.check("bus", v, "CAN bus") && v->isString())
    {
        auto bus = v->asString();

        if (yarp::os::Value * vv; options.check("nodes", vv, "CAN nodes") && vv->isList())
        {
            const auto * nodes = vv->asList();

            std::vector<std::string> names;

            for (int j = 0; j < nodes->size(); j++)
            {
                auto node = nodes->get(j).asString();
                names.push_back(node);
            }

            broker = new SingleBusBroker(bus, names);

            if (!broker->configure(options))
            {
                yCError(CBB) << "Unable to configure broker of CAN bus device" << bus;
                return false;
            }
        }
        else
        {
            yCError(CBB) << R"(Missing key "nodes" or not a list)";
        }
    }
    else
    {
        yCError(CBB) << R"(Missing key "bus" or not a string)";
        return false;
    }

    if (yarp::os::Value * v; options.check("fakeNodes", v, "fake CAN nodes"))
    {
        if (!v->isList())
        {
            yCError(CBB) << R"(Key "fakeNodes" must be a list)";
            return false;
        }

        const auto * ids = v->asList();

        for (int i = 0; i < ids->size(); i++)
        {
            if (!ids->get(i).isInt32())
            {
                yCError(CBB) << "Fake node ID must be an integer:" << ids->get(i).toString();
                return false;
            }

            auto fakeId = "ID" + std::to_string(ids->get(i).asInt32());
            fakeNodes.push(nullptr, fakeId.c_str());
        }
    }

    if (yarp::os::Value * v; options.check("syncPeriod", v, "SYNC message period (s)"))
    {
        auto syncPeriod = v->asFloat64();

        if (syncPeriod <= 0.0)
        {
            yCError(CBB) << "Invalid --syncPeriod:" << syncPeriod;
            return false;
        }

        auto & syncThread = getSyncThread();

        syncThread.registerBroker(broker);
        syncThread.setPeriod(syncPeriod);

        if (yarp::os::Value * vv; options.check("syncObserver", vv, "synchronization signal observer") && vv->isBlob())
        {
            yCDebug(CBB) << "Setting synchronization signal observer";
            auto * observer = *reinterpret_cast<TypedStateObserver<double> * const *>(vv->asBlob());
            syncThread.setObserver(observer);
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

    if (broker)
    {
        delete broker;
        broker = nullptr;
    }

    return ok;
}

// -----------------------------------------------------------------------------
