// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __I_PROXIMITY_SENSORS__
#define __I_PROXIMITY_SENSORS__


namespace roboticslab
{

/**
 *
 * @brief Abstract base for a sensor reader.
 *
 */
class IProximitySensors
{
    public:
        /**
         * Destructor.
         */
        virtual ~IProximitySensors() {};

        virtual bool hasObstacle() = 0;
        virtual bool hasTarget() = 0;

};

}  // namespace roboticslab

#endif  //  __I_PROXIMITY_SENSORS__

