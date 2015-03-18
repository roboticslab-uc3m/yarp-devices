// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "PlaybackServer.hpp"

namespace teo
{

/************************************************************************/
PlaybackServer::PlaybackServer() { }

/************************************************************************/
bool PlaybackServer::configure(ResourceFinder &rf) {

    //-- Parse open options
    std::string webIp = rf.check("webIp",yarp::os::Value(DEFAULT_WEB_IP),"web server ip").asString();
    int webPort = rf.check("webPort",yarp::os::Value(DEFAULT_WEB_PORT),"web server port").asInt();
    std::string name = rf.check("name",Value(DEFAULT_WEB_NAME),"web yarp port name").asString();

    std::string filePath = rf.check("filePath",Value(DEFAULT_FILE_PATH),"robot file path").asString();
    std::string fileExtension = rf.check("fileExtension",Value(DEFAULT_FILE_EXTENSION),"robot file extension").asString();

    int ptModeMs = rf.check("ptModeMs",yarp::os::Value(DEFAULT_PT_MODE_MS),"PT mode miliseconds").asInt();
    CD_INFO("Using ptModeMs: %d (default: %d).\n",ptModeMs,int(DEFAULT_PT_MODE_MS));

    playbackThread.hold = rf.check("hold");

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

    //-- Configure and open web server
    responder.setRf(&rf);
    responder.setPlaybackThread(&playbackThread);
    responder.setFilePath(filePath);
    responder.setFileExtension(fileExtension);
    server.setReader(responder);
    Contact contact = Contact::byName(name);
    if (webPort!=0) {
        contact = contact.addSocket("",webIp,webPort);
    }
    if (!server.open(contact)) return false;
    contact = server.where();

    return true;
}

/************************************************************************/

bool PlaybackServer::updateModule() {
    CD_INFO("Alive...\n");
    return true;
}

/************************************************************************/

bool PlaybackServer::close() {

    return true;
}

/************************************************************************/

}  // namespace teo
