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

    //-- Configure and open web server
    responder.setRf(&rf);
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
