// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __WEB_RESPONDER__
#define __WEB_RESPONDER__

#include <fstream>  //-- For std::ifstream
#include <dirent.h> //-- For listing directory contents
#include <vector>

#include <yarp/os/PortReader.h>
#include <yarp/os/ResourceFinder.h>

#include "ColorDebug.hpp"

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

        void setRf(yarp::os::ResourceFinder* value);

protected:

        yarp::os::ResourceFinder* rf;
        std::string readFile(const std::string& fileName);
        std::vector<std::string> listFromDir(const std::string& dirName);
        std::string& replaceAll(std::string& context, const std::string& from, const std::string& to);

};

}  // namespace teo

#endif





