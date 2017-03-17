// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "ExampleRemoteControlboard.hpp"

namespace teo
{

int ExampleRemoteControlboard::run(int argc, char **argv)
{
    yarp::os::ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultContext("exampleRemoteControlboard");
    rf.setDefaultConfigFile("exampleRemoteControlboard.ini");
    rf.configure(argc, argv);

    std::string robot = rf.check("robot",yarp::os::Value(DEFAULT_ROBOT),"name of /robot to be used").asString();
    printf("\t--robot: %s [%s]\n",robot.c_str(),DEFAULT_ROBOT);

    printf("Note: requires a running instance of teoSim\n");
    if (!yarp::os::Network::checkNetwork())
    {
        printf("Please start a yarp name server first\n");
        return 1;
    }

    //Configure Drivers
    yarp::os::Property options; //create an instance of Property, a nice YARP class for storing name-value (key-value) pairs
    options.put("device","remote_controlboard"); //we add a name-value pair that indicates the YARP device
    options.put("remote",robot); //we add info on to whom we will connect
    options.put("local","/local"); //we add info on how we will call ourselves on the YARP network
    dd.open(options); //Configure the YARP multi-use driver with the given options

    if(!dd.isValid())
    {
        printf("%s not available.\n", robot.c_str());
        dd.close();
        yarp::os::Network::fini(); //disconnect from the YARP network
        return 1;
    }

    bool ok = dd.view(pos); // connect 'pos' interface to 'dd' device
    if (!ok)
    {
        printf("[error] Problems acquiring robot interface\n");
        return 1;
    }
    printf("[success] testAsibot acquired robot interface\n");

    pos->setPositionMode(); //use the object to set the device to position mode (as opposed to velocity mode)

    printf("test positionMove(1,-35)\n");
    pos->positionMove(1, -35);

    printf("Delaying 5 seconds...\n");
    yarp::os::Time::delay(5);

    ok = dd.view(enc); // connect 'enc' interface to 'dd' device
    double d;
    enc->getEncoder(0,&d);
    printf("test getEncoder(0) -> is at: %f\n", d);

    ok = dd.view(vel); // connect 'vel' interface to 'dd' device
    vel->setVelocityMode(); //use the object to set the device to velocity mode (as opposed to position mode)
    printf("test velocityMove(0,10)\n");
    vel->velocityMove(0,10);

    printf("Delaying 5 seconds...\n");
    yarp::os::Time::delay(5);

    dd.close();

    return 0;
}

} //namespace TEO
