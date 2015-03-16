// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "PlaybackServer.hpp"

namespace teo
{

/************************************************************************/

bool WebResponder::read(yarp::os::ConnectionReader &in) {
    //-- Kind of boilerplate for mini web server
    Bottle got, response;
    if (!got.read(in)) return false;
    ConnectionWriter *out = in.getWriter();
    if (out==NULL) return true;
    response.addString("web");

    //--
    printf("Got: %s\n", got.toString().c_str());


    return true;
}

/************************************************************************/

}  // namespace teo

