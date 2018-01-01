// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusHico.hpp"

#include <cstring>
#include <cerrno>

#include <algorithm>
#include <set>
#include <vector>

#include <yarp/os/Time.h>

#include <ColorDebug.hpp>

// -----------------------------------------------------------------------------

namespace
{
    struct not_consecutive
    {
        bool operator()(unsigned int lhs, unsigned int rhs)
        {
            return rhs != lhs + 1;
        }
    };
}

// -----------------------------------------------------------------------------

const int roboticslab::CanBusHico::FilterManager::MAX_FILTERS = 4;

// -----------------------------------------------------------------------------

roboticslab::CanBusHico::FilterManager::FilterManager(int fileDescriptor)
    : fd(fileDescriptor),
      valid(true)
{
}

// -----------------------------------------------------------------------------

bool roboticslab::CanBusHico::FilterManager::parseIds(const yarp::os::Bottle & b)
{
    bool modified = false;

    for (int i = 0; i < b.size(); i++)
    {
        modified |= stage.insert(b.get(i).asInt()).second;
    }

    if (!stage.empty() && modified)
    {
        if (!clearFilters(false))
        {
            return false;
        }

        return bulkUpdate();
    }

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::CanBusHico::FilterManager::hasId(unsigned int id) const
{
    return currentlyActive.find(id) != currentlyActive.end();
}

// -----------------------------------------------------------------------------

bool roboticslab::CanBusHico::FilterManager::isValid() const
{
    return valid;
}

// -----------------------------------------------------------------------------

bool roboticslab::CanBusHico::FilterManager::insertId(unsigned int id)
{
    stage.insert(id);

    if (!clearFilters(false))
    {
        return false;
    }

    return bulkUpdate();
}

// -----------------------------------------------------------------------------

bool roboticslab::CanBusHico::FilterManager::eraseId(unsigned int id)
{
    stage.erase(id);

    if (!clearFilters(false))
    {
        return false;
    }

    return bulkUpdate();
}

// -----------------------------------------------------------------------------

bool roboticslab::CanBusHico::FilterManager::clearFilters(bool clearStage)
{
    CD_DEBUG("(%d)\n", clearStage);

    if (::ioctl(fd, IOC_CLEAR_FILTERS) == -1)
    {
        CD_ERROR("ioctl() error: %s\n", std::strerror(errno));
        return false;
    }

    currentlyActive.clear();

    if (clearStage)
    {
        stage.clear();
    }

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::CanBusHico::FilterManager::setMaskedFilter(unsigned int id)
{
    CD_DEBUG("(%d)\n", id);

    struct can_filter filter;
    filter.type = FTYPE_AMASK;
    filter.mask = 0x7F;  //-- dsPIC style, mask specifies "do care" bits
    filter.code = id;

    if (::ioctl(fd, IOC_SET_FILTER, &filter) == -1)
    {
        CD_ERROR("Could not set filter: %s.\n", std::strerror(errno));
        return false;
    }

    currentlyActive.insert(id);

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::CanBusHico::FilterManager::setRangedFilter(unsigned int lower, unsigned int upper)
{
    CD_DEBUG("(%d, %d)\n", lower, upper);

    struct can_filter filter;
    filter.type = FTYPE_RANGE;
    filter.lower = lower;
    filter.upper = upper;

    if (::ioctl(fd, IOC_SET_FILTER, &filter) == -1)
    {
        CD_ERROR("Could not set filter: %s.\n", std::strerror(errno));
        return false;
    }

    for (unsigned int id = lower; id < upper + 1; id++)
    {
        currentlyActive.insert(id);
    }

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::CanBusHico::FilterManager::bulkUpdate()
{
    std::vector< std::vector<unsigned int> > sequences;

    std::set<unsigned int>::iterator itSeq = stage.begin();
    std::set<unsigned int>::const_iterator itEnd = stage.end();

    while (itSeq != itEnd)
    {
        std::set<unsigned int>::iterator itNext = std::adjacent_find(itSeq, itEnd, not_consecutive());

        if (itNext != itEnd)
        {
            ++itNext;
        }        

        std::vector<unsigned int> sequence(itSeq, itNext);
        sequences.push_back(sequence);
        itSeq = itNext;
    }

    if (sequences.size() > MAX_FILTERS)
    {
        CD_WARNING("MAX_FILTERS exceeded (%d > %d).\n", sequences.size(), MAX_FILTERS);
        valid = false;
    }
    else
    {
        valid = true;

        bool ok = true;

        for (unsigned int i = 0; i < sequences.size(); i++)
        {
            const std::vector<unsigned int> & seq = sequences[i];

            if (seq.size() == 1)
            {
                ok &= setMaskedFilter(seq[0]);
            }
            else
            {
                ok &= setRangedFilter(seq[0], seq[seq.size() - 1]);
            }

            yarp::os::Time::delay(DELAY);
        }

        if (!ok)
        {
            return false;
        }
    }

    return true;
}

// -----------------------------------------------------------------------------
