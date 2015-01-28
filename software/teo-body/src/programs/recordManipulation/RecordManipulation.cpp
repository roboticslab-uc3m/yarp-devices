// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "RecordManipulation.hpp"

/************************************************************************/
RecordManipulation::RecordManipulation() { }

/************************************************************************/
bool RecordManipulation::configure(yarp::os::ResourceFinder &rf) {

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

    //-- left arm --
    std::string leftArmIni = rf.findFileByName("../manipulation/leftArm.ini");

    yarp::os::Property leftArmOptions;
    if (! leftArmOptions.fromConfigFile(leftArmIni) ) {  //-- Put first because defaults to wiping out.
        CD_ERROR("Could not configure from \"leftArm.ini\".\n");
        return false;
    }
    CD_SUCCESS("Configured left arm from %s.\n",leftArmIni.c_str());
    leftArmOptions.put("name","/teo/leftArm");
    leftArmOptions.put("device","bodybot");
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

    yarp::os::Property rightArmOptions;
    if (! rightArmOptions.fromConfigFile(rightArmIni) ) {  //-- Put first because defaults to wiping out.
        CD_ERROR("Could not configure from \"rightArm.ini\".\n");
        return false;
    }
    CD_SUCCESS("Configured right arm from %s.\n",rightArmIni.c_str());
    rightArmOptions.put("name","/teo/rightArm");
    rightArmOptions.put("device","bodybot");
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

    if ( ! leftArmDevice.view( moveGripperThread.leftArmPos ) ) {
        CD_ERROR("Could not obtain left gripperPos.\n");
        return false;
    }
    CD_SUCCESS("Obtained left gripperPos.\n");

    if ( ! rightArmDevice.view( moveGripperThread.rightArmPos ) ) {
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

    ::fclose(filePtr);

    return true;
}

/************************************************************************/
