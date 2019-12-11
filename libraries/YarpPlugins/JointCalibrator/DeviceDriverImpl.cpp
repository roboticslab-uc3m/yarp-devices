// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "JointCalibrator.hpp"

#include <ColorDebug.h>

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
            CD_ERROR("Missing %s/%s key or size mismatch.\n", key.c_str(), keys.c_str());
            return false;
        }

        return true;
    }
}

bool JointCalibrator::open(yarp::os::Searchable & config)
{
    CD_DEBUG("%s\n", config.toString().c_str());

    axes = config.check("joints", yarp::os::Value(0), "number of controlled axes").asInt32();

    if (axes == 0)
    {
        CD_ERROR("Illegal axis count: %d.\n", axes);
        return false;
    }

    return checkAndSet(axes, config, homeSpecs.pos, "home", "homes", "zero position (degrees)")
            && checkAndSet(axes, config, homeSpecs.vel, "homeVel", "homeVels", "zero velocity (degrees/second)")
            && checkAndSet(axes, config, homeSpecs.acc, "homeAcc", "homeAccs", "zero acceleration (degrees/second^2)")
            && checkAndSet(axes, config, parkSpecs.pos, "park", "parks", "zero position (degrees)")
            && checkAndSet(axes, config, parkSpecs.vel, "parkVel", "parkVels", "zero velocity (degrees/second)")
            && checkAndSet(axes, config, parkSpecs.acc, "parkAcc", "parkAccs", "zero acceleration (degrees/second^2)");
}

bool JointCalibrator::close()
{
    return true;
}
