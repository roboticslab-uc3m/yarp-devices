// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __LINEAR_INTERPOLATION_BUFFER_HPP__
#define __LINEAR_INTERPOLATION_BUFFER_HPP__

#include <stdint.h>

#include <yarp/os/Mutex.h>
#include <yarp/os/Searchable.h>

#include "TechnosoftIpos.hpp"

// https://github.com/roboticslab-uc3m/yarp-devices/issues/198#issuecomment-487279910
#define PT_BUFFER_MAX_SIZE 285
#define PVT_BUFFER_MAX_SIZE 222
#define PT_PVT_BUFFER_LOW_SIGNAL 15 // max: 15
#define DEFAULT_LIN_INTERP_MODE "pvt"
#define DEFAULT_LIN_INTERP_BUFFER_SIZE 1

namespace roboticslab
{

class TechnosoftIpos;

/**
 * @ingroup TechnosoftIpos
 */
class LinearInterpolationBuffer
{
public:
    LinearInterpolationBuffer(double periodMs, double bufferSize, TechnosoftIpos * technosoftIpos);
    virtual ~LinearInterpolationBuffer() {}
    void resetIntegrityCounter();
    void setInitialReference(double target);
    void updateTarget(double target);
    virtual void setSubMode(uint8_t * msg) = 0;
    void setBufferSize(uint8_t * msg);
    virtual void createMessage(uint8_t * msg) = 0;
    static LinearInterpolationBuffer * createBuffer(yarp::os::Searchable& config, TechnosoftIpos * technosoftIpos);

protected:
    TechnosoftIpos * technosoftIpos;
    double periodMs;
    double bufferSize;
    double lastSentTarget;
    double lastReceivedTarget;
    int integrityCounter;
    mutable yarp::os::Mutex mutex;
};

/**
 * @ingroup TechnosoftIpos
 */
class PtBuffer : public LinearInterpolationBuffer
{
public:
    PtBuffer(double periodMs, double bufferSize, TechnosoftIpos * technosoftIpos);
    virtual void setSubMode(uint8_t * msg);
    virtual void createMessage(uint8_t * msg);
};

/**
 * @ingroup TechnosoftIpos
 */
class PvtBuffer : public LinearInterpolationBuffer
{
public:
    PvtBuffer(double periodMs, double bufferSize, TechnosoftIpos * technosoftIpos);
    virtual void setSubMode(uint8_t * msg);
    virtual void createMessage(uint8_t * msg);
};

} // namespace roboticslab

#endif // __LINEAR_INTERPOLATION_BUFFER_HPP__
