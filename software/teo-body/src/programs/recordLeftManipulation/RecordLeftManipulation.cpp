// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "RecordLeftManipulation.hpp"

/************************************************************************/
RecordLeftManipulation::RecordLeftManipulation() { }

/************************************************************************/
bool RecordLeftManipulation::configure(yarp::os::ResourceFinder &rf) {

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
    std::string leftArmIni = manipulation_root + "/app/testManipulationBot2/conf/leftArm.ini";

    yarp::os::Property leftArmOptions;
    if (! leftArmOptions.fromConfigFile(leftArmIni) ) {  //-- Put first because defaults to wiping out.
        CD_ERROR("Could not open %s.\n",leftArmIni.c_str());
        return false;
    }
    CD_SUCCESS("Opened %s.\n",leftArmIni.c_str());
    leftArmOptions.put("device","manipulationbot2");

    leftArmDevice.open(leftArmOptions);
    
    if (!leftArmDevice.isValid()) {
        CD_ERROR("leftArmDevice instantiation not worked.\n");
        CD_ERROR("Be sure CMake \"ENABLE_BodyYarp_manipulationbot2\" variable is set \"ON\"\n");
        CD_ERROR("\"SKIP_manipulationbot2 is set\" --> should be --> \"ENABLE_manipulationbot2 is set\"\n");
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

    if ( ! leftGripperDevice.view( recordRateThread.leftGripperEnc ) ) {
        CD_ERROR("Could not obtain leftGripperEnc.\n");
        return false;
    }
    CD_SUCCESS("Obtained leftGripperEnc.\n");

    if ( ! leftGripperDevice.view( moveGripperThread.leftGripperPos ) ) {
        CD_ERROR("Could not obtain left gripperPos.\n");
        return false;
    }
    CD_SUCCESS("Obtained left gripperPos.\n");


    //-- Do stuff.
    CD_INFO("setTorqueMode in 1 second...\n");
    yarp::os::Time::delay(1);

    recordRateThread.leftArmEnc->getAxes( &(recordRateThread.leftArmNumMotors) );
    for(int i=0; i<recordRateThread.leftArmNumMotors; i++)
        recordRateThread.leftArmTrq->setRefTorque(i,0);

    recordRateThread.leftArmTrq->setTorqueMode();

    //-- Start the threads.
    moveGripperThread.setLeftOpenChar('1');
    moveGripperThread.setLeftCloseChar('q');
    moveGripperThread.start();
    CD_INFO("Start thread in 1 second...\n");
    yarp::os::Time::delay(1);
    recordRateThread.setRate(ptModeMs);
    recordRateThread.start();

    return true;
}

/************************************************************************/

bool RecordLeftManipulation::updateModule() {
    //printf("RecordLeftManipulation alive...\n");
    return true;
}

/************************************************************************/

bool RecordLeftManipulation::close() {

    moveGripperThread.stop();

    recordRateThread.stop();

    leftArmDevice.close();

    leftGripperDevice.close();

    ::fclose(filePtr);

    return true;
}

/************************************************************************/
