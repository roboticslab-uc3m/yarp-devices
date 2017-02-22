// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "PlaybackThread.hpp"

namespace teo {

// -----------------------------------------------------------------------------

void PlaybackThread::run()
{

    while ( ! this->isStopping() )
    {
        if ( getState() == PLAYING )
        {
            std::vector<double> row;

            if( ! this->getNext(row) )
            {
                CD_DEBUG("End of rows, stopPlay()\n");
                stopPlay();
                break;
            }

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
        }  // if ( getState() == PLAYING )
    }  // while ( ! this->isStopping() )
}

// -----------------------------------------------------------------------------

void PlaybackThread::onStop()
{
    setState( NOT_PLAYING );
}

// -----------------------------------------------------------------------------

}  // namespace teo
