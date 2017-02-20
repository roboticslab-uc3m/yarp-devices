// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "PlaybackThread.hpp"

namespace teo {

// -----------------------------------------------------------------------------

bool PlaybackThread::open(yarp::os::Searchable& config)
{
    std::string fileName = config.check("file",yarp::os::Value("test.txt"),"file name").asString();
    this->fromFile(fileName);
    return true;
}

// -----------------------------------------------------------------------------

bool PlaybackThread::close()
{
    return true;
}

// -----------------------------------------------------------------------------

}  // namespace teo
