// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "EmulatedControlBoard.hpp"

#include <yarp/os/LogStream.h>
#include <yarp/os/Time.h>

#include "LogComponent.hpp"

#define checkListParam(axes, value, vector) checkListParamInternal((axes), (value), (vector), #value, #vector)

namespace
{
    bool checkListParamInternal(int axes, double value, std::vector<double> & vec, const char * valueStr, const char * vecStr)
    {
        if (vec.empty())
        {
            yCInfo(ECB, "EmulatedControlBoard not using individual %s, defaulting to %s", vecStr, valueStr);
            vec.resize(axes, value);
        }
        else if (vec.size() != axes)
        {
            yCError(ECB, "%s->size() != axes", vecStr);
            return false;
        }

        return true;
    }
}

// ------------------- DeviceDriver Related ------------------------------------

bool EmulatedControlBoard::open(yarp::os::Searchable& config)
{
    if (!parseParams(config))
    {
        yCError(ECB) << "Could not parse parameters";
        return false;
    }

    if (m_mode == "pos")
    {
        controlMode = POSITION_MODE;
    }
    else if (m_mode == "vel")
    {
        controlMode = VELOCITY_MODE;
    }
    else if (m_mode == "posd")
    {
        controlMode = POSITION_DIRECT_MODE;
    }
    else
    {
        yCError(ECB) << "Unrecognized mode:" << m_mode;
        return false;
    }

    if (!checkListParam(m_axes, m_genInitPos, m_initPoss) ||
        !checkListParam(m_axes, m_genJointTol, m_jointTols) ||
        !checkListParam(m_axes, m_genMaxLimit, m_maxLimits) ||
        !checkListParam(m_axes, m_genMinLimit, m_minLimits) ||
        !checkListParam(m_axes, m_genRefSpeed, m_refSpeeds) ||
        !checkListParam(m_axes, m_genEncRawExposed, m_encRawExposeds) ||
        !checkListParam(m_axes, m_genVelRawExposed, m_velRawExposeds))
    {
        return false;
    }

    jointStatus.resize(m_axes, NOT_CONTROLLING);

    encRaw.resize(m_axes, 0.0);
    refAcc.resize(m_axes, 1.0);
    targetExposed.resize(m_axes, 0.0);
    velRaw.resize(m_axes, 0.0);

    for (unsigned int i = 0; i < m_axes; i++)
    {
        setEncoder(i, m_initPoss[i]);
    }

    lastTime = yarp::os::Time::now();

    // Start the PeriodicThread
    return yarp::os::PeriodicThread::setPeriod(m_jmcMs * 0.001) && yarp::os::PeriodicThread::start();
}

// -----------------------------------------------------------------------------

bool EmulatedControlBoard::close()
{
    yarp::os::PeriodicThread::stop();
    return true;
}

// -----------------------------------------------------------------------------
