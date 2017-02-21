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
        if( initTime != initTime )
        {
            initTime = yarp::os::Time::now();
            initRow = row[timeIdx];
        }
        else
        {
            yarp::os::Time::delay( initTime + (row[timeIdx] - initRow) - yarp::os::Time::now() );
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
