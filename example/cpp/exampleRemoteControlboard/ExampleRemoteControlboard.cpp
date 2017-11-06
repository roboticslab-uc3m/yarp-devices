// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "ExampleRemoteControlboard.hpp"

namespace roboticslab
{

int ExampleRemoteControlboard::run(int argc, char **argv)
{
    //-- Init
    yarp::os::ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultContext("exampleRemoteControlboard");
    rf.setDefaultConfigFile("exampleRemoteControlboard.ini");
    rf.configure(argc, argv);

    std::string robot = rf.check("robot",yarp::os::Value(DEFAULT_ROBOT),"name of /robot to be used").asString();
    printf("\t--robot: %s [%s]\n",robot.c_str(),DEFAULT_ROBOT);

    printf("Note: requires a running instance of roboticslabSim\n");
    if (!yarp::os::Network::checkNetwork())
    {
        printf("Please start a yarp name server first\n");
        return 1;
    }

    //-- Configure device
    yarp::os::Property options; //create an instance of Property, a nice YARP class for storing name-value (key-value) pairs
    options.put("device","remote_controlboard"); //we add a name-value pair that indicates the YARP device
    options.put("remote",robot); //we add info on to whom we will connect
    options.put("local","/local"); //we add info on how we will call ourselves on the YARP network
    dd.open(options); //Configure the YARP multi-use driver with the given options
    if( ! dd.isValid() )
    {
        printf("%s not available.\n", robot.c_str());
        dd.close();
        yarp::os::Network::fini(); //disconnect from the YARP network
        return 1;
    }

    //-- View interfaces
    if ( ! dd.view(pos) )  // connect 'pos' interface to 'dd' device
    {
        printf("[error] Problems acquiring position interface\n");
        return 1;
    }
    if ( ! dd.view(pos2) )  // connect 'pos2' interface to 'dd' device
    {
        printf("[error] Problems acquiring position interface\n");
        return 1;
    }
    printf("[success] Acquired position interface\n");

    if ( ! dd.view(enc) ) // connect 'enc' interface to 'dd' device
    {
        printf("[error] Problems acquiring encoder interface\n");
        return 1;
    }
    printf("[success] Acquired encoder interface\n");

    if ( ! dd.view(vel) ) // connect 'vel' interface to 'dd' device
    {
        printf("[error] Problems acquiring velocity interface\n");
        return 1;
    }
    printf("[success] Acquired velocity interface\n");

    //-- Start

    /*printf("setPositionMode()\n");
    pos->setPositionMode(); //use the position object to set the device to position mode (as opposed to velocity mode)

    printf("setRefSpeed(0,5)\n");
    pos->setRefSpeed(0,5);

    printf("setRefAcceleration(0,5)\n");
    pos->setRefAcceleration(0,5);

    printf("positionMove(0,-10)\n");
    pos->positionMove(0, -10);

    printf("Wait to reach");
    bool done = false;
    do
    {
        yarp::os::Time::delay(0.1);
        pos->checkMotionDone( & done );
        printf(".");
        fflush(stdout);
    }
    while( ! done );
    printf("\n");

    double d;
    enc->getEncoder(0,&d);
    printf("getEncoder(0) -> is at: %f\n", d);

    printf("setVelocityMode()\n");
    vel->setVelocityMode(); //use the object to set the device to velocity mode (as opposed to position mode)

    printf("velocityMove(0,5)\n");
    vel->velocityMove(0,5);

    printf("Delaying 2 seconds...\n");
    yarp::os::Time::delay(2);

    printf("velocityMove(0,0) <- stop\n");
    vel->velocityMove(0,0);

    printf("setPositionMode()\n");
    pos->setPositionMode(); //use the position object to set the device to position mode (as opposed to velocity mode)*/

    /*{
        printf("positionMove() <- -3,-3\n");
        std::vector<double> q(2,0.0);
        q[0] = -3.0;
        q[1] = -3.0;
        pos->positionMove( q.data() );
        printf("Wait to reach");
        bool done = false;
        while(!done)
        {
            pos->checkMotionDone( & done );
            printf(".");
            fflush(stdout);
            yarp::os::Time::delay(0.1);
        }
        printf("\n");

        std::vector<double> d(2,0.0);
        enc->getEncoders( d.data() );
        printf("getEncoders() -> is at: %f %f\n", d[0], d[1]);
    }

    {
        printf("positionMove() <- -0,-0\n");
        std::vector<double> q(2,0.0);
        q[0] = -3.0;
        q[1] = -3.0;
        pos->positionMove( q.data() );
        printf("Wait to reach");
        bool done = false;
        while(!done)
        {
            pos->checkMotionDone( & done );
            printf(".");
            fflush(stdout);
            yarp::os::Time::delay(0.1);
        }
        printf("\n");

        std::vector<double> d(2,0.0);
        enc->getEncoders( d.data() );
        printf("getEncoders() -> is at: %f %f\n", d[0], d[1]);
    }*/

    {
        printf("positionMove() <- -3,-3\n");
        std::vector<double> q(2,0.0);
        std::vector<int> mask(2,1);
        q[0] = -3.0;
        q[1] = -3.0;
        pos2->positionMove( 2, mask.data(), q.data() );
        printf("Wait to reach");
        bool done = false;
        while(!done)
        {
            pos->checkMotionDone( & done );
            printf(".");
            fflush(stdout);
            yarp::os::Time::delay(0.1);
        }
        printf("\n");

        std::vector<double> d(2,0.0);
        enc->getEncoders( d.data() );
        printf("getEncoders() -> is at: %f %f\n", d[0], d[1]);
    }


    dd.close();

    return 0;
}

} //namespace roboticslab
