// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "JointCalibrator.hpp"

#include <yarp/os/LogStream.h>

#include "LogComponent.hpp"

namespace
{
    bool checkAndSet(int axes, double global, const std::vector<double> & individual, std::vector<double> & out)
    {
        if (!individual.empty())
        {
            if (individual.size() != axes)
            {
                yCError(JC) << "Invalid size for individual values:" << individual.size() << "instead of" << axes;
                return false;
            }

            out = individual;
        }
        else
        {
            out.assign(axes, global);
        }

        return true;
    }
}

bool JointCalibrator::open(yarp::os::Searchable & config)
{
    if (!parseParams(config))
    {
        yCError(JC) << "Unable to parse parameters";
        return false;
    }

    if (m_joints == 0)
    {
        yCError(JC) << "Illegal axis count:" << m_joints;
        return false;
    }

    return checkAndSet(m_joints, m_home, m_homes, homeSpecs.pos)
        && checkAndSet(m_joints, m_homeVel, m_homeVels, homeSpecs.vel)
        && checkAndSet(m_joints, m_homeAcc, m_homeAccs, homeSpecs.acc)
        && checkAndSet(m_joints, m_park, m_parks, parkSpecs.pos)
        && checkAndSet(m_joints, m_parkVel, m_parkVels, parkSpecs.vel)
        && checkAndSet(m_joints, m_parkAcc, m_parkAccs, parkSpecs.acc);
}

bool JointCalibrator::close()
{
    return true;
}
