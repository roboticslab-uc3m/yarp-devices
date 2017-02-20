// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "PlaybackThread.hpp"

namespace teo {

// -----------------------------------------------------------------------------

bool PlaybackThread::play()
{
    std::vector<double> row;

    int rowCounter = 0;
    while( this->getNext(row) )
    {
        std::cout << "Row[" << rowCounter << "]: ";
        for(int i=0;i<row.size();i++)
        {
            std::cout << row[i] << " ";
        }
        std::cout << std::endl;
        yarp::os::Time::delay(row[0]);
        rowCounter++;
    }

    return true;
}

// -----------------------------------------------------------------------------

}  // namespace teo
