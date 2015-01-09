// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "RecordManipulation.hpp"

/************************************************************************/
RecordManipulation::RecordManipulation() { }

/************************************************************************/
bool RecordManipulation::configure(yarp::os::ResourceFinder &rf) {

    //CD_INFO("Using name: %s.\n", rf.find("name").asString().c_str() );

    std::string manipulation_root = ::getenv("MANIPULATION_ROOT");
    CD_INFO("Using root: %s.\n", manipulation_root.c_str() );

    int ptModeMs = rf.check("ptModeMs",yarp::os::Value(DEFAULT_PT_MODE_MS),"PT mode miliseconds").asInt();
    CD_INFO("Using ptModeMs: %d (default: %d).\n",ptModeMs,int(DEFAULT_PT_MODE_MS));

    //-- Open file for writing.
    std::string fileName = rf.check("file",yarp::os::Value(DEFAULT_FILE_NAME),"file name").asString();
    CD_INFO("Using file: %s (default: " DEFAULT_FILE_NAME ").\n",fileName.c_str());
    filePtr = ::fopen (fileName.c_str(),"w");
    if( ! filePtr ) {
        CD_PERROR("Could not open file: %s.\n",fileName.c_str());
        return false;
    }
    CD_SUCCESS("Opened file: %s.\n",fileName.c_str());

    //-- Open manipulator devices.
    std::string leftArmIni = manipulation_root + "/app/testBodyBot/conf/leftArm.ini";

    yarp::os::Property leftArmOptions;
    if (! leftArmOptions.fromConfigFile(leftArmIni) ) {  //-- Put first because defaults to wiping out.
        CD_ERROR("Could not open %s.\n",leftArmIni.c_str());
        return false;
    }
    CD_SUCCESS("Opened %s.\n",leftArmIni.c_str());
    leftArmOptions.put("device","bodybot");

    leftArmDevice.open(leftArmOptions);
    
    if (!leftArmDevice.isValid()) {
        CD_ERROR("leftArmDevice instantiation not worked.\n");
        CD_ERROR("Be sure CMake \"ENABLE_BodyYarp_bodybot\" variable is set \"ON\"\n");
        CD_ERROR("\"SKIP_bodybot is set\" --> should be --> \"ENABLE_bodybot is set\"\n");
        // robotDevice.close();  // un-needed?
        return false;
    }

    std::string rightArmIni = manipulation_root + "/app/testBodyBot/conf/rightArm.ini";

    yarp::os::Property rightArmOptions;
    if (! rightArmOptions.fromConfigFile(rightArmIni) ) {  //-- Put first because defaults to wiping out.
        CD_ERROR("Could not open %s.\n",rightArmIni.c_str());
        return false;
    }
    CD_SUCCESS("Opened %s.\n",rightArmIni.c_str());
    rightArmOptions.put("name","/teo/rightArm");
    rightArmOptions.put("device","bodybot");

    rightArmDevice.open(rightArmOptions);

    if (!rightArmDevice.isValid()) {
        CD_ERROR("rightArmDevice instantiation not worked.\n");
        CD_ERROR("Be sure CMake \"ENABLE_BodyYarp_bodybot\" variable is set \"ON\"\n");
        CD_ERROR("\"SKIP_bodybot is set\" --> should be --> \"ENABLE_bodybot is set\"\n");
        // robotDevice.close();  // un-needed?
        return false;
    }

    std::string leftGripperIni = manipulation_root + "/app/testGripperBot/conf/leftGripper.ini";

    yarp::os::Property leftGripperOptions;
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

    yarp::os::Property rightGripperOptions;
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
    recordRateThread.setFilePtr(filePtr);

    //-- Obtain manipulator interfaces.
    if ( ! leftArmDevice.view( recordRateThread.leftArmEnc ) ) {
        CD_ERROR("Could not obtain leftArmEnc.\n");
        return false;
    }
    CD_SUCCESS("Obtained leftArmEnc.\n");

    if ( ! leftArmDevice.view( recordRateThread.leftArmTrq ) ) {
        CD_ERROR("Could not obtain leftArmTrq.\n");
        return false;
    }
    CD_SUCCESS("Obtained leftArmTrq.\n");

    if ( ! rightArmDevice.view( recordRateThread.rightArmEnc ) ) {
        CD_ERROR("Could not obtain rightArmEnc.\n");
        return false;
    }
    CD_SUCCESS("Obtained rightArmEnc.\n");

    if ( ! rightArmDevice.view( recordRateThread.rightArmTrq ) ) {
        CD_ERROR("Could not obtain rightArmTrq.\n");
        return false;
    }
    CD_SUCCESS("Obtained rightArmTrq.\n");

    if ( ! leftGripperDevice.view( recordRateThread.leftGripperEnc ) ) {
        CD_ERROR("Could not obtain leftGripperEnc.\n");
        return false;
    }
    CD_SUCCESS("Obtained leftGripperEnc.\n");

    if ( ! rightGripperDevice.view( recordRateThread.rightGripperEnc ) ) {
        CD_ERROR("Could not obtain rightGripperEnc.\n");
        return false;
    }
    CD_SUCCESS("Obtained rightGripperEnc.\n");

    if ( ! leftGripperDevice.view( moveGripperThread.leftGripperPos ) ) {
        CD_ERROR("Could not obtain left gripperPos.\n");
        return false;
    }
    CD_SUCCESS("Obtained left gripperPos.\n");

    if ( ! rightGripperDevice.view( moveGripperThread.rightGripperPos ) ) {
        CD_ERROR("Could not obtain right gripperPos.\n");
        return false;
    }
    CD_SUCCESS("Obtained right gripperPos.\n");


    //-- Do stuff.
    CD_INFO("setTorqueMode in 1 second...\n");
    yarp::os::Time::delay(1);

    recordRateThread.leftArmEnc->getAxes( &(recordRateThread.leftArmNumMotors) );
    for(int i=0; i<recordRateThread.leftArmNumMotors; i++)
        recordRateThread.leftArmTrq->setRefTorque(i,0);

    recordRateThread.rightArmEnc->getAxes( &(recordRateThread.rightArmNumMotors) );
    for(int i=0; i< recordRateThread.rightArmNumMotors; i++)
        recordRateThread.rightArmTrq->setRefTorque(i,0);

    recordRateThread.leftArmTrq->setTorqueMode();
    recordRateThread.rightArmTrq->setTorqueMode();

    //-- Start the threads.
    moveGripperThread.setLeftOpenChar('1');
    moveGripperThread.setLeftCloseChar('q');
    moveGripperThread.setRightOpenChar('2');
    moveGripperThread.setRightCloseChar('w');
    moveGripperThread.start();
    CD_INFO("Start thread in 1 second...\n");
    yarp::os::Time::delay(1);
    recordRateThread.setRate(ptModeMs);
    recordRateThread.start();

    return true;
}

/************************************************************************/

bool RecordManipulation::updateModule() {
    //printf("RecordManipulation alive...\n");
    return true;
}

/************************************************************************/

bool RecordManipulation::close() {

    moveGripperThread.stop();

    recordRateThread.stop();

    leftArmDevice.close();
    rightArmDevice.close();

    leftGripperDevice.close();
    rightGripperDevice.close();

    ::fclose(filePtr);

    return true;
}

/************************************************************************/
