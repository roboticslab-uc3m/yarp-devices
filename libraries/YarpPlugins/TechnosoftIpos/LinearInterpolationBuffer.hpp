// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __LINEAR_INTERPOLATION_BUFFER_HPP__
#define __LINEAR_INTERPOLATION_BUFFER_HPP__

#include <stdint.h>

#include <yarp/os/Mutex.h>

#include "TechnosoftIpos.hpp"

namespace roboticslab
{

class TechnosoftIpos;

/**
 * @ingroup TechnosoftIpos
 */
class LinearInterpolationBuffer
{
public:
    LinearInterpolationBuffer(double periodMs, TechnosoftIpos * technosoftIpos);
    virtual ~LinearInterpolationBuffer() {}
    void setInitialReference(double target);
    void updateTarget(double target);
    virtual void createMessage(uint8_t * msg) = 0;

protected:
    TechnosoftIpos * technosoftIpos;
    double periodMs;
    double lastSentTarget;
    double lastReceivedTarget;
    mutable yarp::os::Mutex mutex;
};

/**
 * @ingroup TechnosoftIpos
 */
class PtBuffer : public LinearInterpolationBuffer
{
public:
    PtBuffer(double periodMs, TechnosoftIpos * technosoftIpos);
    virtual void createMessage(uint8_t * msg);
};

/**
 * @ingroup TechnosoftIpos
 */
class PvtBuffer : public LinearInterpolationBuffer
{
public:
    PvtBuffer(double periodMs, TechnosoftIpos * technosoftIpos);
    virtual void createMessage(uint8_t * msg);
};

} // namespace roboticslab

#endif // __LINEAR_INTERPOLATION_BUFFER_HPP__
