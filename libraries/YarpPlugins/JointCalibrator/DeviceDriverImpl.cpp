// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "JointCalibrator.hpp"

#include <yarp/os/LogStream.h>

#include "LogComponent.hpp"

using namespace roboticslab;

namespace
{
    bool checkAndSet(int axes, const yarp::os::Searchable & config, std::vector<double> & v,
            const std::string & key, const std::string & keys, const std::string & comment)
    {
        if (config.check(key, "(shared) " + comment))
        {
            v.assign(axes, config.find(key).asFloat64());
        }
        else if (config.check(keys, comment))
        {
            yarp::os::Bottle * parsed = config.find(keys).asList();

            for (auto i = 0; i < parsed->size(); i++)
            {
                v.push_back(parsed->get(i).asFloat64());
            }
        }
        else
        {
            yCError(JC) << "Missing" << key << "/" << keys << "key or size mismatch";
            return false;
        }

        return true;
    }
}

bool JointCalibrator::open(yarp::os::Searchable & config)
{
    axes = config.check("joints", yarp::os::Value(0), "number of controlled axes").asInt32();
    isBlocking = config.check("block", yarp::os::Value(false), "commands should block").asBool();

    if (axes == 0)
    {
        yCError(JC) << "Illegal axis count:" << axes;
        return false;
    }

    return checkAndSet(axes, config, homeSpecs.pos, "home", "homes", "zero position (degrees)")
            && checkAndSet(axes, config, homeSpecs.vel, "homeVel", "homeVels", "zero velocity (degrees/second)")
            && checkAndSet(axes, config, homeSpecs.acc, "homeAcc", "homeAccs", "zero acceleration (degrees/second^2)")
            && checkAndSet(axes, config, parkSpecs.pos, "park", "parks", "park position (degrees)")
            && checkAndSet(axes, config, parkSpecs.vel, "parkVel", "parkVels", "park velocity (degrees/second)")
            && checkAndSet(axes, config, parkSpecs.acc, "parkAcc", "parkAccs", "park acceleration (degrees/second^2)");
}

bool JointCalibrator::close()
{
    return true;
}
