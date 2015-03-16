// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __WEB_RESPONDER__
#define __WEB_RESPONDER__

#include <yarp/os/PortReader.h>

namespace teo
{

/**
 *
 * @ingroup webResponder
 *
 * @brief Manages YARP web RPCs as callbacks and additionally manages resouces
 * and parses HTML code before acting as a mini-server.
 *
 */
class WebResponder : public yarp::os::PortReader {

    public:

        bool read(yarp::os::ConnectionReader& in);

    protected:

        std::string readFile(const std::string& filePath);

};

}  // namespace teo

#endif





