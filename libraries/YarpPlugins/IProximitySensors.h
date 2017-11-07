// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __I_PROXIMITY_SENSORS__
#define __I_PROXIMITY_SENSORS__

namespace roboticslab
{

/**
 *
 * @brief Abstract base class for a sensor reader.
 *
 */
class IProximitySensors
{
public:

    enum alert_level { ZERO, LOW, HIGH };

    virtual ~IProximitySensors() {};

    virtual alert_level getAlertLevel() = 0;

    virtual bool hasTarget() = 0;
};

}  // namespace roboticslab

#endif  //  __I_PROXIMITY_SENSORS__
