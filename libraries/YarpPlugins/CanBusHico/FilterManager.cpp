// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusHico.hpp"

#include <cstring>
#include <cerrno>

#include <algorithm>
#include <set>
#include <vector>

#include <yarp/os/LogStream.h>
#include <yarp/os/Time.h>

#include "LogComponent.hpp"

using namespace roboticslab;

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

CanBusHico::FilterManager::FilterManager(const CanBusHico & owner, int fileDescriptor, bool enableRanges)
    : owner(owner),
      fd(fileDescriptor),
      valid(true),
      enableRanges(enableRanges)
{
}

// -----------------------------------------------------------------------------

bool CanBusHico::FilterManager::parseIds(const yarp::os::Bottle & b)
{
    bool modified = false;

    for (int i = 0; i < b.size(); i++)
    {
        modified |= stage.insert(b.get(i).asInt32()).second;
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

bool CanBusHico::FilterManager::hasId(unsigned int id) const
{
    return currentlyActive.find(id) != currentlyActive.end();
}

// -----------------------------------------------------------------------------

bool CanBusHico::FilterManager::isValid() const
{
    return valid;
}

// -----------------------------------------------------------------------------

bool CanBusHico::FilterManager::insertId(unsigned int id)
{
    stage.insert(id);

    if (!clearFilters(false))
    {
        return false;
    }

    return bulkUpdate();
}

// -----------------------------------------------------------------------------

bool CanBusHico::FilterManager::eraseId(unsigned int id)
{
    stage.erase(id);

    if (!clearFilters(false))
    {
        return false;
    }

    return bulkUpdate();
}

// -----------------------------------------------------------------------------

bool CanBusHico::FilterManager::clearFilters(bool clearStage)
{
    if (::ioctl(fd, IOC_CLEAR_FILTERS) == -1)
    {
        yCIError(HICO, owner.id()) << "ioctl() error while clearing filters:" << std::strerror(errno);
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

bool CanBusHico::FilterManager::setMaskedFilter(unsigned int id)
{
    struct can_filter filter;
    filter.type = FTYPE_AMASK;
    filter.mask = 0x7F;  //-- dsPIC style, mask specifies "do care" bits
    filter.code = id;

    if (::ioctl(fd, IOC_SET_FILTER, &filter) == -1)
    {
        yCIError(HICO, owner.id()) << "Could not set filter:" << std::strerror(errno);
        return false;
    }

    currentlyActive.insert(id);

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::CanBusHico::FilterManager::setRangedFilter(unsigned int lower, unsigned int upper)
{
    struct can_filter filter;
    filter.type = FTYPE_RANGE;
    filter.lower = lower;
    filter.upper = upper;

    if (::ioctl(fd, IOC_SET_FILTER, &filter) == -1)
    {
        yCIError(HICO, owner.id()) << "Could not set filter:" << std::strerror(errno);
        return false;
    }

    for (unsigned int id = lower; id < upper + 1; id++)
    {
        currentlyActive.insert(id);
    }

    return true;
}

// -----------------------------------------------------------------------------

bool CanBusHico::FilterManager::bulkUpdate()
{
    std::vector<std::vector<unsigned int>> sequences;

    if (enableRanges)
    {
        auto itSeq = stage.begin();
        auto itEnd = stage.cend();

        while (itSeq != itEnd)
        {
            auto itNext = std::adjacent_find(itSeq, itEnd, not_consecutive());

            if (itNext != itEnd)
            {
                ++itNext;
            }

            sequences.emplace_back(itSeq, itNext);
            itSeq = itNext;
        }
    }
    else
    {
        for (auto it = stage.cbegin(); it != stage.cend(); ++it)
        {
            sequences.emplace_back(1, *it);
        }
    }

    if (sequences.size() > MAX_FILTERS)
    {
        yCIWarning(HICO, owner.id(), "MAX_FILTERS exceeded (%zu > %d)", sequences.size(), MAX_FILTERS);
        valid = false;
    }
    else
    {
        valid = true;

        bool ok = true;

        for (unsigned int i = 0; i < sequences.size(); i++)
        {
            const std::vector<unsigned int> & seq = sequences[i];

            if (enableRanges && seq.size() > 1)
            {
                ok &= setRangedFilter(seq[0], seq[seq.size() - 1]);
            }
            else
            {
                for (unsigned int j = 0; i < seq.size(); j++)
                {
                    ok &= setMaskedFilter(seq[j]);
                }
            }
        }

        if (!ok)
        {
            return false;
        }
    }

    return true;
}

// -----------------------------------------------------------------------------
