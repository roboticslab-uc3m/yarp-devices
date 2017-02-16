// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __I_PLAYBACK__
#define __I_PLAYBACK__

#include <vector>

namespace teo
{

/**
 *
 * @brief Abstract base for Playback.
 *
 */
class IPlayback
{
public:
    /**
     * Destructor.
     */
    virtual ~IPlayback() {}

    virtual bool getNumRows(int* num) = 0;

    virtual bool getNext(std::vector<double>& row) = 0;

};

}  // namespace teo

#endif  //  __I_PLAYBACK__
