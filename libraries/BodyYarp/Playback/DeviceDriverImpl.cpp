// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "Playback.hpp"

namespace teo
{

// -----------------------------------------------------------------------------

bool Playback::open(yarp::os::Searchable& config)
{
    std::string fileName = config.check("file",yarp::os::Value("in.txt"),"file name").asString();

    file.open(fileName.c_str());
    if( ! file.is_open() )
    {
          printf("Not able to open file.\n");
          return false;
    }

    return true;
}

// -----------------------------------------------------------------------------

bool Playback::close()
{
    return true;
}

// -----------------------------------------------------------------------------

}  // namespace teo
