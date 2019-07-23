// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __LINEAR_INTERPOLATION_BUFFER_HPP__
#define __LINEAR_INTERPOLATION_BUFFER_HPP__

#include <stdint.h>

#include <yarp/os/Mutex.h>
#include <yarp/os/Searchable.h>

// https://github.com/roboticslab-uc3m/yarp-devices/issues/198#issuecomment-487279910
#define PT_BUFFER_MAX_SIZE 285
#define PVT_BUFFER_MAX_SIZE 222
#define PT_PVT_BUFFER_LOW_SIGNAL 15 // max: 15
#define DEFAULT_LIN_INTERP_MODE "pvt"

namespace roboticslab
{

/**
 * @ingroup TechnosoftIpos
 */
class LinearInterpolationBuffer
{
public:
    LinearInterpolationBuffer();
    virtual ~LinearInterpolationBuffer() {}
    void resetIntegrityCounter();
    virtual void setInitialReference(double target);
    void updateTarget(double target);
    int getBufferSize() const;
    virtual void configureSubMode(uint8_t * msg) = 0;
    void configureBufferSize(uint8_t * msg);
    virtual void configureMessage(uint8_t * msg) = 0;
    static LinearInterpolationBuffer * createBuffer(const yarp::os::Searchable & config);

protected:
    double periodMs;
    int bufferSize;
    double factor;
    double maxVel;
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
    PtBuffer();
    virtual void configureSubMode(uint8_t * msg);
    virtual void configureMessage(uint8_t * msg);

private:
    double maxDistance;
};

/**
 * @ingroup TechnosoftIpos
 */
class PvtBuffer : public LinearInterpolationBuffer
{
public:
    PvtBuffer();
    virtual void setInitialReference(double target);
    virtual void configureSubMode(uint8_t * msg);
    virtual void configureMessage(uint8_t * msg);

private:
    double previousTarget;
    bool isFirstPoint;
};

} // namespace roboticslab

#endif // __LINEAR_INTERPOLATION_BUFFER_HPP__
