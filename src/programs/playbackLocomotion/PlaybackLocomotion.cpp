// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "PlaybackLocomotion.hpp"

/************************************************************************/
PlaybackLocomotion::PlaybackLocomotion() { }

/************************************************************************/
bool PlaybackLocomotion::configure(ResourceFinder &rf) {

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

    //-- left leg --
    std::string leftLegIni = rf.findFileByName("../locomotion/leftLeg.ini");

    yarp::os::Property leftLegOptions;
    if (! leftLegOptions.fromConfigFile(leftLegIni) ) {  //-- Put first because defaults to wiping out.
        CD_ERROR("Could not configure from \"leftLeg.ini\".\n");
        return false;
    }
    CD_SUCCESS("Configured left leg from %s.\n",leftLegIni.c_str());
    leftLegOptions.put("name","/teo/leftLeg");
    leftLegOptions.put("device","bodybot");
    leftLegOptions.put("ptModeMs",ptModeMs);
    if (rf.check("home")) leftLegOptions.put("home",1);
    if (rf.check("reset")) leftLegOptions.put("reset",1);

    leftLegDevice.open(leftLegOptions);
    
    if (!leftLegDevice.isValid()) {
        CD_ERROR("leftLegDevice instantiation not worked.\n");
        CD_ERROR("Be sure CMake \"ENABLE_LocomotionYarp_bodybot\" variable is set \"ON\"\n");
        CD_ERROR("\"SKIP_bodybot is set\" --> should be --> \"ENABLE_bodybot is set\"\n");
        // robotDevice.close();  // un-needed?
        return false;
    }

    //-- right leg --
    std::string rightLegIni = rf.findFileByName("../locomotion/rightLeg.ini");

    yarp::os::Property rightLegOptions;
    if (! rightLegOptions.fromConfigFile(rightLegIni) ) {  //-- Put first because defaults to wiping out.
        CD_ERROR("Could not configure from \"rightLeg.ini\".\n");
        return false;
    }
    CD_SUCCESS("Configured right leg from %s.\n",rightLegIni.c_str());
    rightLegOptions.put("name","/teo/rightLeg");
    rightLegOptions.put("device","bodybot");
    rightLegOptions.put("ptModeMs",ptModeMs);
    if (rf.check("home")) rightLegOptions.put("home",1);
    if (rf.check("reset")) rightLegOptions.put("reset",1);

    rightLegDevice.open(rightLegOptions);

    if (!rightLegDevice.isValid()) {
        CD_ERROR("rightLegDevice instantiation not worked.\n");
        CD_ERROR("Be sure CMake \"ENABLE_LocomotionYarp_bodybot\" variable is set \"ON\"\n");
        CD_ERROR("\"SKIP_bodybot is set\" --> should be --> \"ENABLE_bodybot is set\"\n");
        // robotDevice.close();  // un-needed?
        return false;
    }

    //-- Configure the thread.

    //-- Obtain manipulator interfaces.
    if ( ! leftLegDevice.view( playbackThread.leftLegPos ) ) {
        CD_ERROR("Could not obtain leftLegPos.\n");
        return false;
    }
    CD_SUCCESS("Obtained leftLegPos.\n");

    if ( ! leftLegDevice.view( playbackThread.leftLegPosDirect ) ) {
        CD_ERROR("Could not obtain leftLegPosDirect.\n");
        return false;
    }
    CD_SUCCESS("Obtained leftLegPosDirect.\n");

    if ( ! rightLegDevice.view( playbackThread.rightLegPos ) ) {
        CD_ERROR("Could not obtain leftLegPos.\n");
        return false;
    }
    CD_SUCCESS("Obtained rightLegPos.\n");

    if ( ! rightLegDevice.view( playbackThread.rightLegPosDirect ) ) {
        CD_ERROR("Could not obtain rightLegPosDirect.\n");
        return false;
    }
    CD_SUCCESS("Obtained rightLegPosDirect.\n");

    //-- Do stuff.
    playbackThread.leftLegPos->getAxes( &(playbackThread.leftLegNumMotors) );
    playbackThread.rightLegPos->getAxes( &(playbackThread.rightLegNumMotors) );

    CD_INFO("setPositionDirectMode...\n");

    playbackThread.leftLegPosDirect->setPositionDirectMode();
    playbackThread.rightLegPosDirect->setPositionDirectMode();

    //-- Start the thread.
    CD_INFO("Start thread...\n");
    playbackThread.start();

    return true;
}

/************************************************************************/

bool PlaybackLocomotion::updateModule() {
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

bool PlaybackLocomotion::close() {

    playbackThread.stop();

    leftLegDevice.close();
    rightLegDevice.close();

    playbackThread.ifs.close();

    return true;
}

/************************************************************************/
