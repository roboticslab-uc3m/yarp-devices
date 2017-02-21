// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "PlaybackThread.hpp"

namespace teo {

// -----------------------------------------------------------------------------

bool PlaybackThread::play()
{
    std::vector<double> row;

    int rowCounter = 0;
    double now = yarp::os::Time::now();
    while( this->getNext(row) )
    {
        if( timeToSubtract != timeToSubtract )
        {
            CD_DEBUG("now: %f, timestamp: %f, diff: %f\n",now,row[timeIdx],now-row[timeIdx])
            timeToSubtract = now - row[timeIdx];
        }
        else
        {
            double wait = timeToSubtract - row[timeIdx];
            CD_DEBUG("timestamp: %f, diff: %f\n",timeToSubtract,row[timeIdx],wait);
            yarp::os::Time::delay( wait );
        }

        std::cout << "Row[" << rowCounter << "]: ";
        for(int i=0;i<row.size();i++)
        {
            std::cout << row[i] << " ";
        }
        std::cout << std::endl;
        rowCounter++;
    }

    return true;
}

// -----------------------------------------------------------------------------

}  // namespace teo
