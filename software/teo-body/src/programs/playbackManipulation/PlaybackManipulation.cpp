// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "PlaybackManipulation.hpp"

/************************************************************************/
PlaybackManipulation::PlaybackManipulation() { }

/************************************************************************/
bool PlaybackManipulation::configure(ResourceFinder &rf) {

    int ptModeMs = rf.check("ptModeMs",yarp::os::Value(DEFAULT_PT_MODE_MS),"PT mode miliseconds").asInt();
    CD_INFO("Using ptModeMs: %d (default: %d).\n",ptModeMs,int(DEFAULT_PT_MODE_MS));

    //-- Open file for reading.
    std::string fileName = rf.check("file",Value(DEFAULT_FILE_NAME),"file name").asString();
    CD_INFO("Using file: %s (default: " DEFAULT_FILE_NAME ").\n",fileName.c_str());
    playbackThread.ifs.open( fileName.c_str() );
    if( ! playbackThread.ifs.is_open() ) {
        CD_ERROR("Could not open file: %s.\n",fileName.c_str());
        return false;
    }
    CD_SUCCESS("Opened file: %s.\n",fileName.c_str());

    //-- left arm --
    std::string leftArmIni = rf.findFileByName("../manipulation/leftArm.ini");

    Property leftArmOptions;
    if (! leftArmOptions.fromConfigFile(leftArmIni) ) {  //-- Put first because defaults to wiping out.
        CD_ERROR("Could not configure from \"leftArm.ini\".\n");
        return false;
    }
    CD_SUCCESS("Configured left arm from %s.\n",leftArmIni.c_str());
    leftArmOptions.put("name","/teo/leftArm");
    leftArmOptions.put("device","bodybot");
    leftArmOptions.put("ptModeMs",ptModeMs);

    leftArmDevice.open(leftArmOptions);
    
    if (!leftArmDevice.isValid()) {
        CD_ERROR("leftArmDevice instantiation not worked.\n");
        CD_ERROR("Be sure CMake \"ENABLE_BodyYarp_bodybot\" variable is set \"ON\"\n");
        CD_ERROR("\"SKIP_bodybot is set\" --> should be --> \"ENABLE_bodybot is set\"\n");
        // robotDevice.close();  // un-needed?
        return false;
    }

    //-- right arm --
    std::string rightArmIni = rf.findFileByName("../manipulation/rightArm.ini");

    Property rightArmOptions;
    if (! rightArmOptions.fromConfigFile(rightArmIni) ) {  //-- Put first because defaults to wiping out.
        CD_ERROR("Could not configure from \"rightArm.ini\".\n");
        return false;
    }
    CD_SUCCESS("Configured right arm from %s.\n",rightArmIni.c_str());
    rightArmOptions.put("name","/teo/rightArm");
    rightArmOptions.put("device","bodybot");
    rightArmOptions.put("ptModeMs",ptModeMs);

    rightArmDevice.open(rightArmOptions);

    if (!rightArmDevice.isValid()) {
        CD_ERROR("rightArmDevice instantiation not worked.\n");
        CD_ERROR("Be sure CMake \"ENABLE_BodyYarp_bodybot\" variable is set \"ON\"\n");
        CD_ERROR("\"SKIP_bodybot is set\" --> should be --> \"ENABLE_bodybot is set\"\n");
        // robotDevice.close();  // un-needed?
        return false;
    }

    //-- Configure the thread.

    //-- Obtain manipulator interfaces.
    if ( ! leftArmDevice.view( playbackThread.leftArmPos ) ) {
        CD_ERROR("Could not obtain leftArmPos.\n");
        return false;
    }
    CD_SUCCESS("Obtained leftArmPos.\n");

    if ( ! leftArmDevice.view( playbackThread.leftArmPosDirect ) ) {
        CD_ERROR("Could not obtain leftArmPosDirect.\n");
        return false;
    }
    CD_SUCCESS("Obtained leftArmPosDirect.\n");

    if ( ! leftArmDevice.view( playbackThread.leftArmEncs ) ) {
        CD_ERROR("Could not obtain leftArmEncs.\n");
        return false;
    }
    CD_SUCCESS("Obtained rightArmEncs.\n");


    if ( ! rightArmDevice.view( playbackThread.rightArmPos ) ) {
        CD_ERROR("Could not obtain leftArmPos.\n");
        return false;
    }
    CD_SUCCESS("Obtained rightArmPos.\n");

    if ( ! rightArmDevice.view( playbackThread.rightArmPosDirect ) ) {
        CD_ERROR("Could not obtain rightArmPosDirect.\n");
        return false;
    }
    CD_SUCCESS("Obtained rightArmPosDirect.\n");

    if ( ! rightArmDevice.view( playbackThread.rightArmEncs ) ) {
        CD_ERROR("Could not obtain rightArmEncs.\n");
        return false;
    }
    CD_SUCCESS("Obtained rightArmEncs.\n");


    //-- Do stuff.
    playbackThread.leftArmPos->getAxes( &(playbackThread.leftArmNumMotors) );
    playbackThread.rightArmPos->getAxes( &(playbackThread.rightArmNumMotors) );

    std::vector<double> leftArmEncValues(playbackThread.leftArmNumMotors);
    playbackThread.leftArmEncs->getEncoders(leftArmEncValues.data());
    CD_DEBUG("leftArmEncValues: ");
    for(int i=0;i<playbackThread.leftArmNumMotors;i++)
        CD_DEBUG_NO_HEADER("%f ",leftArmEncValues[i]);
    CD_DEBUG_NO_HEADER("\n");

    std::vector<double> rightArmEncValues(playbackThread.rightArmNumMotors);
    playbackThread.rightArmEncs->getEncoders(rightArmEncValues.data());
    CD_DEBUG("rightArmEncs: ");
    for(int i=0;i<playbackThread.rightArmNumMotors;i++)
        CD_DEBUG_NO_HEADER("%f ",rightArmEncValues[i]);
    CD_DEBUG_NO_HEADER("\n");

    CD_INFO("setPositionDirectMode in 1 second...\n");
    yarp::os::Time::delay(1);

    playbackThread.leftArmPosDirect->setPositionDirectMode();
    playbackThread.rightArmPosDirect->setPositionDirectMode();

    //-- Start the thread.
    CD_INFO("Start thread in 1 second...\n");
    yarp::os::Time::delay(1);
    playbackThread.start();

    return true;
}

/************************************************************************/

bool PlaybackManipulation::updateModule() {
    if( ! playbackThread.isRunning() ) {
        CD_DEBUG("playbackThread not running, so stopping module...\n");
        yarp::os::Time::delay(1);
        this->stopModule();
        return true;
    }
    CD_DEBUG("playbackThread running...\n");
    return true;
}

/************************************************************************/

bool PlaybackManipulation::close() {

    playbackThread.stop();

    leftArmDevice.close();
    rightArmDevice.close();

    playbackThread.ifs.close();

    return true;
}

/************************************************************************/
