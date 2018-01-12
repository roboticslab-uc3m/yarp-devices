// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "FakeControlboard.hpp"

#include <yarp/os/Bottle.h>
#include <yarp/os/Time.h>

#include <ColorDebug.hpp>

// ------------------- DeviceDriver Related ------------------------------------

bool roboticslab::FakeControlboard::open(yarp::os::Searchable& config)
{
    axes = config.check("axes", DEFAULT_AXES, "number of axes to control").asInt();
    jmcMs = config.check("jmcMs", DEFAULT_JMC_MS, "period of JMC rate thread").asInt();

    double genInitPos = config.check("genInitPos", DEFAULT_GEN_INIT_POS, "general initialization positions").asDouble();
    double genJointTol = config.check("genJointTol", DEFAULT_GEN_JOINT_TOL, "general joint tolerances").asDouble();
    double genMaxLimit = config.check("genMaxLimit", DEFAULT_GEN_MAX_LIMIT, "general max limits").asDouble();
    double genMinLimit = config.check("genMinLimit", DEFAULT_GEN_MIN_LIMIT, "general min limits").asDouble();
    double genRefSpeed = config.check("genRefSpeed", DEFAULT_GEN_REF_SPEED, "general ref speed").asDouble();
    double genEncRawExposed = config.check("genEncRawExposed", DEFAULT_GEN_ENC_RAW_EXPOSED, "general EncRawExposed").asDouble();
    double genVelRawExposed = config.check("genVelRawExposed", DEFAULT_GEN_VEL_RAW_EXPOSED, "general VelRawExposed").asDouble();
    
    int modePosVelInt = config.check("modePosVel", DEFAULT_MODE_POS_VEL, "0:pos, 1:vel").asInt();

    switch (modePosVelInt)
    {
    case 0:
        modePosVel = POSITION_MODE;
        break;
    case 1:
        modePosVel = VELOCITY_MODE;
        break;
    default:
        CD_ERROR("Unrecognized mode identifier: %d (0:pos, 1:vel).\n", modePosVelInt);
        return false;
    }

    yarp::os::Bottle* initPoss;

    if (config.check("initPoss"))
    {
        initPoss = config.find("initPoss").asList();
        CD_INFO("FakeControlboard using individual initPoss: %s\n", initPoss->toString().c_str());

        if ((unsigned)initPoss->size() != axes)
        {
            CD_WARNING("initPoss->size() != axes\n");
        }
    }
    else
    {
        initPoss = 0;
        CD_INFO("FakeControlboard not using individual initPoss, defaulting to genInitPos.\n");
    }

    yarp::os::Bottle* jointTols;

    if (config.check("jointTols"))
    {
        jointTols = config.find("jointTols").asList();
        CD_INFO("FakeControlboard using individual jointTols: %s\n", jointTols->toString().c_str());

        if ((unsigned)jointTols->size() != axes)
        {
            CD_WARNING("jointTols->size() != axes\n");
        }
    }
    else
    {
        jointTols = 0;
        CD_INFO("FakeControlboard not using individual jointTols, defaulting to genJointTol.\n");
    }

    yarp::os::Bottle* maxLimits;

    if (config.check("maxLimits"))
    {
        maxLimits = config.find("maxLimits").asList();
        CD_INFO("FakeControlboard using individual maxLimits: %s\n", maxLimits->toString().c_str());

        if ((unsigned)maxLimits->size() != axes)
        {
            CD_WARNING("maxLimits->size() != axes\n");
        }
    }
    else
    {
        maxLimits = 0;
        CD_INFO("FakeControlboard not using individual maxLimits, defaulting to genMaxLimit.\n");
    }

    yarp::os::Bottle* minLimits;

    if (config.check("minLimits"))
    {
        minLimits = config.find("minLimits").asList();
        CD_INFO("FakeControlboard using individual minLimits: %s\n", minLimits->toString().c_str());
        
        if ((unsigned)minLimits->size() != axes)
        {
            CD_WARNING("minLimits->size() != axes\n");
        }
    }
    else
    {
        minLimits = 0;
        CD_INFO("FakeControlboard not using individual minLimits, defaulting to genMinLimit.\n");
    }

    yarp::os::Bottle* refSpeeds;

    if (config.check("refSpeeds"))
    {
        refSpeeds = config.find("refSpeeds").asList();
        CD_INFO("FakeControlboard using individual refSpeeds: %s\n", refSpeeds->toString().c_str());
    
        if ((unsigned)refSpeeds->size() != axes)
        {
            CD_WARNING("refSpeeds->size() != axes\n");
        }
    }
    else
    {
        refSpeeds = 0;
        CD_INFO("FakeControlboard not using individual refSpeeds, defaulting to genRefSpeed.\n");
    }

    yarp::os::Bottle* encRawExposeds;

    if (config.check("encRawExposeds"))
    {
        encRawExposeds = config.find("encRawExposeds").asList();
        CD_INFO("FakeControlboard using individual encRawExposeds: %s\n", encRawExposeds->toString().c_str());
     
        if ((unsigned)encRawExposeds->size() != axes)
        {
            CD_WARNING("encRawExposeds->size() != axes\n");
        }
    }
    else
    {
        encRawExposeds = 0;
        CD_INFO("FakeControlboard not using individual encRawExposeds, defaulting to genEncRawExposed.\n");
    }

    yarp::os::Bottle* velRawExposeds;

    if (config.check("velRawExposeds"))
    {
        velRawExposeds = config.find("velRawExposeds").asList();
        CD_INFO("FakeControlboard using individual velRawExposeds: %s\n", velRawExposeds->toString().c_str());

        if ((unsigned)velRawExposeds->size() != axes)
        {
            CD_WARNING("velRawExposeds->size() != axes\n");
        }
    }
    else
    {
        velRawExposeds = 0;
        CD_INFO("FakeControlboard not using individual velRawExposeds, defaulting to genVelRawExposed.\n");
    }

    encRawExposed.resize(axes);
    jointStatus.resize(axes);
    initPos.resize(axes);
    jointTol.resize(axes);
    maxLimit.resize(axes);
    minLimit.resize(axes);
    refSpeed.resize(axes);
    velRawExposed.resize(axes);

    for (unsigned int i = 0; i < axes; i++)
    {
        jointStatus[i] = NOT_MOVING;

        refSpeed[i]      = refSpeeds      ? refSpeeds->get(i).asDouble()      : genRefSpeed;
        minLimit[i]      = minLimits      ? minLimits->get(i).asDouble()      : genMinLimit;
        maxLimit[i]      = maxLimits      ? maxLimits->get(i).asDouble()      : genMaxLimit;
        initPos[i]       = initPoss       ? initPoss->get(i).asDouble()       : genInitPos;
        jointTol[i]      = jointTols      ? jointTols->get(i).asDouble()      : genJointTol;
        encRawExposed[i] = encRawExposeds ? encRawExposeds->get(i).asDouble() : genEncRawExposed;
        velRawExposed[i] = velRawExposeds ? velRawExposeds->get(i).asDouble() : genVelRawExposed;
    }

    encRaw.resize(axes, 0.0);
    refAcc.resize(axes, 1.0);
    targetExposed.resize(axes, 0.0);
    velRaw.resize(axes, 0.0);

    for (unsigned int i = 0; i < axes; i++)
    {
        setEncoder(i, initPos[i]);
    }

    lastTime = yarp::os::Time::now();

    // Start the RateThread
    RateThread::setRate(jmcMs);
    RateThread::start();
    
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::FakeControlboard::close()
{
    RateThread::stop();
    CD_INFO("[FakeControlboard] close()\n");
    return true;
}

// -----------------------------------------------------------------------------
