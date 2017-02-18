// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "Playback.hpp"

namespace teo
{

// -----------------------------------------------------------------------------

bool Playback::getNumRows(int* num)
{
    *num = doublesOnFile.size();

    return true;
}

// -----------------------------------------------------------------------------

bool Playback::getNext(std::vector<double>& row)
{
    if( doublesOnFileIter >= doublesOnFile.size() )
        return false;

    row = doublesOnFile[doublesOnFileIter];

    doublesOnFileIter++;

    return true;
}

// -----------------------------------------------------------------------------

bool Playback::parseFileLine(std::vector<double>& doublesOnFileLine)
{
    doublesOnFileLine.clear();

    if( file.eof() )
        return false;

    std::string s;
    getline(file, s);
    std::stringstream ss(s);

    double d;
    while (ss >> d)
        doublesOnFileLine.push_back(d);

    return true;
}

// -----------------------------------------------------------------------------

}  // namespace teo
