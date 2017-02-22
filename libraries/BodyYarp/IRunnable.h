// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __I_RUNNABLE__
#define __I_RUNNABLE__

namespace teo
{

/**
 *
 * @brief Abstract base for Runnable.
 *
 */
class IRunnable
{
public:
    /**
     * Destructor.
     */
    virtual ~IRunnable() {}

    virtual bool run(std::vector<double>& v) = 0;
};

}  // namespace teo

#endif  //  __I_RUNNABLE__
