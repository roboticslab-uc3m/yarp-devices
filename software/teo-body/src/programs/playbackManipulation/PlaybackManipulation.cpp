// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "PlaybackManipulation.hpp"

/************************************************************************/
PlaybackManipulation::PlaybackManipulation() { }

/************************************************************************/
bool PlaybackManipulation::configure(ResourceFinder &rf) {

    //CD_INFO("Using name: %s.\n", rf.find("name").asString().c_str() );

    std::string manipulation_root = ::getenv("MANIPULATION_ROOT");
    CD_INFO("Using root: %s.\n", manipulation_root.c_str() );

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

    //-- Open manipulator devices.
    std::string leftArmIni = manipulation_root + "/app/testBodyBot/conf/leftArm.ini";

    Property leftArmOptions;
    if (! leftArmOptions.fromConfigFile(leftArmIni) ) {  //-- Put first because defaults to wiping out.
        CD_ERROR("Could not open %s.\n",leftArmIni.c_str());
        return false;
    }
    CD_SUCCESS("Opened %s.\n",leftArmIni.c_str());
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

    std::string rightArmIni = manipulation_root + "/app/testBodyBot/conf/rightArm.ini";

    Property rightArmOptions;
    if (! rightArmOptions.fromConfigFile(rightArmIni) ) {  //-- Put first because defaults to wiping out.
        CD_ERROR("Could not open %s.\n",rightArmIni.c_str());
        return false;
    }
    CD_SUCCESS("Opened %s.\n",rightArmIni.c_str());
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

    std::string leftGripperIni = manipulation_root + "/app/testGripperBot/conf/leftGripper.ini";

    Property leftGripperOptions;
    if (! leftGripperOptions.fromConfigFile(leftGripperIni) ) {  //-- Put first because defaults to wiping out.
        CD_ERROR("Could not open %s.\n",leftGripperIni.c_str());
        return false;
    }
    CD_SUCCESS("Opened %s.\n",leftGripperIni.c_str());
    leftGripperOptions.put("name","/teo/leftGripper");
    leftGripperOptions.put("device","controlboard");
    leftGripperOptions.put("subdevice","gripperbot");

    leftGripperDevice.open(leftGripperOptions);

    if (!leftGripperDevice.isValid()) {
        CD_ERROR("leftGripperDevice instantiation not worked.\n");
        CD_ERROR("Be sure CMake \"ENABLE_BodyYarp_gripperbot\" variable is set \"ON\"\n");
        CD_ERROR("\"SKIP_gripperbot is set\" --> should be --> \"ENABLE_gripperbot is set\"\n");
        // robotDevice.close();  // un-needed?
        return false;
    }

    std::string rightGripperIni = manipulation_root + "/app/testGripperBot/conf/rightGripper.ini";

    Property rightGripperOptions;
    if (! rightGripperOptions.fromConfigFile(rightGripperIni) ) {  //-- Put first because defaults to wiping out.
        CD_ERROR("Could not open %s.\n",rightGripperIni.c_str());
        return false;
    }
    CD_SUCCESS("Opened %s.\n",rightGripperIni.c_str());
    rightGripperOptions.put("name","/teo/rightGripper");
    rightGripperOptions.put("device","controlboard");
    rightGripperOptions.put("subdevice","gripperbot");

    rightGripperDevice.open(rightGripperOptions);

    if (!rightGripperDevice.isValid()) {
        CD_ERROR("rightGripperDevice instantiation not worked.\n");
        CD_ERROR("Be sure CMake \"ENABLE_BodyYarp_gripperbot\" variable is set \"ON\"\n");
        CD_ERROR("\"SKIP_gripperbot is set\" --> should be --> \"ENABLE_gripperbot is set\"\n");
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


    if ( ! leftGripperDevice.view( playbackThread.leftGripperPos ) ) {
        CD_ERROR("Could not obtain leftGripperPos.\n");
        return false;
    }
    CD_SUCCESS("Obtained leftGripperPos.\n");

    if ( ! rightGripperDevice.view( playbackThread.rightGripperPos ) ) {
        CD_ERROR("Could not obtain rightGripperPos.\n");
        return false;
    }
    CD_SUCCESS("Obtained rightGripperPos.\n");


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

    leftGripperDevice.close();
    rightGripperDevice.close();

    playbackThread.ifs.close();

    return true;
}

/************************************************************************/
