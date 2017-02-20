// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __PLAYBACK__
#define __PLAYBACK__

#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <sstream>

#include <fstream>

//#define CD_FULL_FILE  //-- Can be globally managed from father CMake. Good for debugging with polymorphism.
//#define CD_HIDE_DEBUG  //-- Can be globally managed from father CMake.
//#define CD_HIDE_SUCCESS  //-- Can be globally managed from father CMake.
//#define CD_HIDE_INFO  //-- Can be globally managed from father CMake.
//#define CD_HIDE_WARNING  //-- Can be globally managed from father CMake.
//#define CD_HIDE_ERROR  //-- Can be globally managed from father CMake.
#include "ColorDebug.hpp"

#define DEFAULT_RATE_MS 20.0
#define DEFAULT_NUM_CHANNELS 6

namespace teo
{

/**
 * @ingroup BodyYarp
 * \defgroup Playback
 * @brief Contains teo::Playback.
 */

 /**
 * @ingroup Playback
 * @brief Implementation for Playback.
 *
 */
class Playback
{

    public:

        Playback() {
        }

        bool fromFile(const std::string& fileName);

        bool getNumRows(int* num);
        bool getNext(std::vector<double>& row);

    private:

        std::ifstream file;

        bool parseFileLine(std::vector<double>& doublesOnFileLine);

        std::vector< std::vector<double> > doublesOnFile;
        int doublesOnFileIter;

};

}  // namespace teo

#endif  // __PLAYBACK__

