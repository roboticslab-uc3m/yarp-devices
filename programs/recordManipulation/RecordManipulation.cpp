// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "RecordManipulation.hpp"

namespace teo
{

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

    //-- Gripper port
    moveGripperPort.open("/teo/grippers");
    moveGripperPort.useCallback();

    std::string allIni = rf.findFileByName("../launchManipulation/launchManipulation.ini");
    yarp::os::Property allOptions;
    if (! allOptions.fromConfigFile(allIni) ) {  //-- Put first because defaults to wiping out.
        CD_ERROR("Could not configure from \"launchManipulation.ini\".\n");
        return false;
    }
    //CD_SUCCESS("Configuring from %s.\n",allOptions.toString().c_str());

    //-- /dev/can0 --
    yarp::os::Bottle devCan0 = allOptions.findGroup("devCan0");
    CD_DEBUG("%s\n",devCan0.toString().c_str());
    yarp::os::Property optionsDevCan0;
    optionsDevCan0.fromString(devCan0.toString());
    optionsDevCan0.put("device","CanBusControlboard");
    if (rf.check("home")) optionsDevCan0.put("home",1);
    if (rf.check("reset")) optionsDevCan0.put("reset",1);

    leftArmDevice.open(optionsDevCan0);
    
    if (!leftArmDevice.isValid()) {
        CD_ERROR("leftArmDevice instantiation not worked.\n");
        CD_ERROR("Be sure CMake \"ENABLE_BodyYarp_CanBusControlboard\" variable is set \"ON\"\n");
        CD_ERROR("\"SKIP_CanBusControlboard is set\" --> should be --> \"ENABLE_CanBusControlboard is set\"\n");
        // robotDevice.close();  // un-needed?
        return false;
    }

    //-- /dev/can1 --
    yarp::os::Bottle devCan1 = allOptions.findGroup("devCan1");
    CD_DEBUG("%s\n",devCan1.toString().c_str());
    yarp::os::Property optionsDevCan1;
    optionsDevCan1.fromString(devCan1.toString());
    optionsDevCan1.put("device","CanBusControlboard");
    if (rf.check("home")) optionsDevCan1.put("home",1);
    if (rf.check("reset")) optionsDevCan1.put("reset",1);

    rightArmDevice.open(optionsDevCan1);

    if (!rightArmDevice.isValid()) {
        CD_ERROR("rightArmDevice instantiation not worked.\n");
        CD_ERROR("Be sure CMake \"ENABLE_BodyYarp_CanBusControlboard\" variable is set \"ON\"\n");
        CD_ERROR("\"SKIP_CanBusControlboard is set\" --> should be --> \"ENABLE_CanBusControlboard is set\"\n");
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

    if ( ! leftArmDevice.view( moveGripperPort.iPositionControlLeft ) ) {
        CD_ERROR("Could not obtain left gripperPos.\n");
        return false;
    }
    CD_SUCCESS("Obtained left gripperPos.\n");

    if ( ! rightArmDevice.view( moveGripperPort.iPositionControlRight ) ) {
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

    moveGripperPort.disableCallback();
    moveGripperPort.close();

    recordRateThread.stop();

    leftArmDevice.close();
    rightArmDevice.close();

    ::fclose(filePtr);

    return true;
}

/************************************************************************/

}  // namespace teo
