// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "Playback.hpp"

namespace teo
{

// -----------------------------------------------------------------------------

bool Playback::fromFile(const std::string& fileName)
{
    file.open(fileName.c_str());
    if( ! file.is_open() )
    {
          CD_ERROR("Not able to open file: %s\n",fileName.c_str());
          return false;
    }
    CD_SUCCESS("Opened file: %s\n",fileName.c_str());

    std::vector<double> doublesOnFileLine;

    while( this->parseFileLine(doublesOnFileLine) )
    {
        if ( doublesOnFileLine.size() == 0 ) continue;

        doublesOnFile.push_back( doublesOnFileLine );
    }

    doublesOnFileIter = 0;

    file.close();

    return true;
}

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
