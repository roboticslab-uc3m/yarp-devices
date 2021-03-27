// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "EmulatedControlboard.hpp"

#include <yarp/os/Bottle.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Time.h>

// ------------------- DeviceDriver Related ------------------------------------

bool roboticslab::EmulatedControlboard::open(yarp::os::Searchable& config)
{
    yDebug() << "EmulatedControlboard config:" << config.toString();

    axes = config.check("axes", yarp::os::Value(DEFAULT_AXES), "number of axes to control").asInt32();
    jmcMs = config.check("jmcMs", yarp::os::Value(DEFAULT_JMC_MS), "period of JMC periodic thread (milliseconds)").asInt32();

    double genInitPos = config.check("genInitPos", yarp::os::Value(DEFAULT_GEN_INIT_POS), "general initialization positions (meters or degrees)").asFloat64();
    double genJointTol = config.check("genJointTol", yarp::os::Value(DEFAULT_GEN_JOINT_TOL), "general joint tolerances (meters or degrees)").asFloat64();
    double genMaxLimit = config.check("genMaxLimit", yarp::os::Value(DEFAULT_GEN_MAX_LIMIT), "general max limits (meters or degrees)").asFloat64();
    double genMinLimit = config.check("genMinLimit", yarp::os::Value(DEFAULT_GEN_MIN_LIMIT), "general min limits (meters or degrees)").asFloat64();
    double genRefSpeed = config.check("genRefSpeed", yarp::os::Value(DEFAULT_GEN_REF_SPEED), "general ref speed (meters/second or degrees/second)").asFloat64();
    double genEncRawExposed = config.check("genEncRawExposed", yarp::os::Value(DEFAULT_GEN_ENC_RAW_EXPOSED), "general EncRawExposed (meters or degrees)").asFloat64();
    double genVelRawExposed = config.check("genVelRawExposed", yarp::os::Value(DEFAULT_GEN_VEL_RAW_EXPOSED), "general VelRawExposed (meters/second or degrees/second)").asFloat64();
    
    int modePosVelInt = config.check("modePosVel", yarp::os::Value(DEFAULT_MODE_POS_VEL), "0:pos, 1:vel").asInt32();

    switch (modePosVelInt)
    {
    case 0:
        controlMode = POSITION_MODE;
        break;
    case 1:
        controlMode = VELOCITY_MODE;
        break;
    default:
        yError() << "Unrecognized mode identifier:" << modePosVelInt << "(0:pos, 1:vel)";
        return false;
    }

    yarp::os::Bottle* initPoss;

    if (config.check("initPoss", "list of initialization positions (meters or degrees)"))
    {
        initPoss = config.find("initPoss").asList();
        yInfo() << "Using individual initPoss:" << initPoss->toString();

        if ((unsigned)initPoss->size() != axes)
        {
            yWarning() << "initPoss->size() != axes";
        }
    }
    else
    {
        initPoss = 0;
        yInfo() << "EmulatedControlboard not using individual initPoss, defaulting to genInitPos";
    }

    yarp::os::Bottle* jointTols;

    if (config.check("jointTols", "list of joint tolerances (meters or degrees)"))
    {
        jointTols = config.find("jointTols").asList();
        yInfo() << "EmulatedControlboard using individual jointTols:" << jointTols->toString();

        if ((unsigned)jointTols->size() != axes)
        {
            yWarning() << "jointTols->size() != axes";
        }
    }
    else
    {
        jointTols = 0;
        yInfo() << "EmulatedControlboard not using individual jointTols, defaulting to genJointTol";
    }

    yarp::os::Bottle* maxLimits;

    if (config.check("maxLimits", "list of max limits (meters or degrees)"))
    {
        maxLimits = config.find("maxLimits").asList();
        yInfo() << "EmulatedControlboard using individual maxLimits:" << maxLimits->toString();

        if ((unsigned)maxLimits->size() != axes)
        {
            yWarning() << "maxLimits->size() != axes";
        }
    }
    else
    {
        maxLimits = 0;
        yInfo() << "EmulatedControlboard not using individual maxLimits, defaulting to genMaxLimit";
    }

    yarp::os::Bottle* minLimits;

    if (config.check("minLimits", "list of min limits (meters or degrees)"))
    {
        minLimits = config.find("minLimits").asList();
        yInfo() << "EmulatedControlboard using individual minLimits:" << minLimits->toString();
        
        if ((unsigned)minLimits->size() != axes)
        {
            yWarning() << "minLimits->size() != axes";
        }
    }
    else
    {
        minLimits = 0;
        yInfo() << "EmulatedControlboard not using individual minLimits, defaulting to genMinLimit";
    }

    yarp::os::Bottle* refSpeeds;

    if (config.check("refSpeeds", "list of ref speeds (meters/second or degrees/second)"))
    {
        refSpeeds = config.find("refSpeeds").asList();
        yInfo() << "EmulatedControlboard using individual refSpeeds:" << refSpeeds->toString();
    
        if ((unsigned)refSpeeds->size() != axes)
        {
            yWarning() << "refSpeeds->size() != axes";
        }
    }
    else
    {
        refSpeeds = 0;
        yInfo() << "EmulatedControlboard not using individual refSpeeds, defaulting to genRefSpeed";
    }

    yarp::os::Bottle* encRawExposeds;

    if (config.check("encRawExposeds", "list of EncRawExposeds (meters or degrees)"))
    {
        encRawExposeds = config.find("encRawExposeds").asList();
        yInfo() << "EmulatedControlboard using individual encRawExposeds:" << encRawExposeds->toString();
     
        if ((unsigned)encRawExposeds->size() != axes)
        {
            yWarning() << "encRawExposeds->size() != axes";
        }
    }
    else
    {
        encRawExposeds = 0;
        yInfo() << "EmulatedControlboard not using individual encRawExposeds, defaulting to genEncRawExposed";
    }

    yarp::os::Bottle* velRawExposeds;

    if (config.check("velRawExposeds", "list of VelRawExposed (meters/second or degrees/second)"))
    {
        velRawExposeds = config.find("velRawExposeds").asList();
        yInfo() << "EmulatedControlboard using individual velRawExposeds:" << velRawExposeds->toString();

        if ((unsigned)velRawExposeds->size() != axes)
        {
            yWarning() << "velRawExposeds->size() != axes";
        }
    }
    else
    {
        velRawExposeds = 0;
        yInfo() << "EmulatedControlboard not using individual velRawExposeds, defaulting to genVelRawExposed";
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
        jointStatus[i] = NOT_CONTROLLING;

        refSpeed[i]      = refSpeeds      ? refSpeeds->get(i).asFloat64()      : genRefSpeed;
        minLimit[i]      = minLimits      ? minLimits->get(i).asFloat64()      : genMinLimit;
        maxLimit[i]      = maxLimits      ? maxLimits->get(i).asFloat64()      : genMaxLimit;
        initPos[i]       = initPoss       ? initPoss->get(i).asFloat64()       : genInitPos;
        jointTol[i]      = jointTols      ? jointTols->get(i).asFloat64()      : genJointTol;
        encRawExposed[i] = encRawExposeds ? encRawExposeds->get(i).asFloat64() : genEncRawExposed;
        velRawExposed[i] = velRawExposeds ? velRawExposeds->get(i).asFloat64() : genVelRawExposed;
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

    // Start the PeriodicThread
    PeriodicThread::setPeriod(jmcMs * 0.001);
    PeriodicThread::start();
    
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::EmulatedControlboard::close()
{
    PeriodicThread::stop();
    return true;
}

// -----------------------------------------------------------------------------
