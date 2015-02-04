// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "PlaybackManipulation.hpp"

/************************************************************************/
PlaybackManipulation::PlaybackManipulation() { }

/************************************************************************/
bool PlaybackManipulation::configure(ResourceFinder &rf) {

    int ptModeMs = rf.check("ptModeMs",yarp::os::Value(DEFAULT_PT_MODE_MS),"PT mode miliseconds").asInt();
    CD_INFO("Using ptModeMs: %d (default: %d).\n",ptModeMs,int(DEFAULT_PT_MODE_MS));

    playbackThread.hold = rf.check("hold");

    //-- Open file for logging.
    if ( rf.check("log") ) {
        playbackThread.log = true;
        std::string fileName = rf.check("log",yarp::os::Value(DEFAULT_LOG_FILE_NAME),"file name").asString();
        CD_INFO("Using log file: %s (default: " DEFAULT_LOG_FILE_NAME ").\n",fileName.c_str());
        playbackThread.logFilePtr = ::fopen (fileName.c_str(),"w");
        if( ! playbackThread.logFilePtr ) {
            CD_PERROR("Could not open log file: %s.\n",fileName.c_str());
            return false;
        }
        CD_SUCCESS("Opened log file: %s.\n",fileName.c_str());
    } else {
        playbackThread.log = false;
    }

    //-- Open file for reading.
    std::string fileName = rf.check("file",Value(DEFAULT_READ_FILE_NAME),"file name").asString();
    CD_INFO("Using read file: %s (default: " DEFAULT_READ_FILE_NAME ").\n",fileName.c_str());
    playbackThread.ifs.open( fileName.c_str() );
    if( ! playbackThread.ifs.is_open() ) {
        CD_ERROR("Could not open read file: %s.\n",fileName.c_str());
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
    if (rf.check("home")) leftArmOptions.put("home",1);
    if (rf.check("reset")) leftArmOptions.put("reset",1);

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
    if (rf.check("home")) rightArmOptions.put("home",1);
    if (rf.check("reset")) rightArmOptions.put("reset",1);

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

    if ( ! leftArmDevice.view( playbackThread.leftArmEncTimed ) ) {
        CD_ERROR("Could not obtain leftArmEncTimed.\n");
        return false;
    }
    CD_SUCCESS("Obtained leftArmEncTimed.\n");

    if ( ! leftArmDevice.view( playbackThread.leftArmTorque ) ) {
        CD_ERROR("Could not obtain leftArmTorque.\n");
        return false;
    }
    CD_SUCCESS("Obtained leftArmTorque.\n");

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

    if ( ! rightArmDevice.view( playbackThread.rightArmEncTimed ) ) {
        CD_ERROR("Could not obtain rightArmEncTimed.\n");
        return false;
    }
    CD_SUCCESS("Obtained rightArmEncTimed.\n");

    if ( ! rightArmDevice.view( playbackThread.rightArmTorque ) ) {
        CD_ERROR("Could not obtain rightArmTorque.\n");
        return false;
    }
    CD_SUCCESS("Obtained rightArmTorque.\n");

    //-- Do stuff.
    playbackThread.leftArmPos->getAxes( &(playbackThread.leftArmNumMotors) );
    playbackThread.rightArmPos->getAxes( &(playbackThread.rightArmNumMotors) );

    CD_INFO("setPositionDirectMode...\n");
    playbackThread.leftArmPosDirect->setPositionDirectMode();
    playbackThread.rightArmPosDirect->setPositionDirectMode();

    //-- Start the thread.
    CD_INFO("Start thread...\n");
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
    ::fclose(playbackThread.logFilePtr);

    return true;
}

/************************************************************************/
