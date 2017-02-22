// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "PlaybackThread.hpp"

namespace teo {

// -----------------------------------------------------------------------------

bool PlaybackThread::open(yarp::os::Searchable& config)
{
    std::string fileName = config.check("file",yarp::os::Value(DEFAULT_FILE_NAME),"file name").asString();
    timeIdx = config.check("timeIdx",yarp::os::Value(DEFAULT_TIME_IDX),"index of timestamp").asInt();

    CD_INFO("file: %s [%s]\n", fileName.c_str(), DEFAULT_FILE_NAME);
    CD_INFO("timeIdx: %d [%d]\n", timeIdx, DEFAULT_TIME_IDX);

    yarp::os::Bottle mask = config.findGroup("mask").tail();

    rowCounter = 0;

    this->fromFile(fileName);

    setState( NOT_PLAYING );
    this->start();

    return true;
}

// -----------------------------------------------------------------------------

bool PlaybackThread::close()
{
    this->stop();
    return true;
}

// -----------------------------------------------------------------------------

}  // namespace teo
