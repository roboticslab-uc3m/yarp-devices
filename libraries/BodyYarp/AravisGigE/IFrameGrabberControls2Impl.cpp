#include "AravisGigE.hpp"


bool roboticslab::AravisGigE::getCameraDescription(CameraDescriptor *camera)
{
    CD_ERROR("Not implemented!\n");
    return false;
}

bool roboticslab::AravisGigE::hasFeature(int feature, bool *hasFeature)
{
    CD_DEBUG("Request to know if camera has feature %d\n", feature);
    if (yarp_arv_feature_map.find(feature) != yarp_arv_feature_map.end())
        *hasFeature = true;
    else
        *hasFeature = false;

    return true;
}

bool roboticslab::AravisGigE::setFeature(int feature, double value)
{
    CD_ERROR("Requested to set feature %d, but not implemented!\n", feature);
    return false;
}

bool roboticslab::AravisGigE::getFeature(int feature, double *value)
{
    CD_DEBUG("Property with yarp id %d requested\n", feature);
    auto yarp_feature = yarp_arv_feature_map.find(feature);
    if (yarp_feature == yarp_arv_feature_map.end())
    {
        CD_ERROR("Property with yarp id %d not available\n", feature);
        return false;
    }

    *value = arv_device_get_integer_feature_value(arv_camera_get_device(camera), yarp_feature->second);
    CD_DEBUG("Value: %d.\n", *value);
    return true;
}

bool roboticslab::AravisGigE::setFeature(int feature, double value1, double value2)
{
    CD_ERROR("Not implemented!\n");
    return false;
}

bool roboticslab::AravisGigE::getFeature(int feature, double *value1, double *value2)
{
    CD_ERROR("Not implemented!\n");
    return false;
}

bool roboticslab::AravisGigE::hasOnOff(int feature, bool *HasOnOff)
{
    CD_ERROR("Not implemented!\n");
    return false;
}

bool roboticslab::AravisGigE::setActive(int feature, bool onoff)
{
    CD_ERROR("Not implemented!\n");
    return false;
}

bool roboticslab::AravisGigE::getActive(int feature, bool *isActive)
{
    CD_ERROR("Not implemented!\n");
    return false;
}

bool roboticslab::AravisGigE::hasAuto(int feature, bool *hasAuto)
{
    CD_ERROR("Not implemented!\n");
    return false;
}

bool roboticslab::AravisGigE::hasManual(int feature, bool *hasManual)
{
    CD_ERROR("Not implemented!\n");
    return false;
}

bool roboticslab::AravisGigE::hasOnePush(int feature, bool *hasOnePush)
{
    CD_ERROR("Not implemented!\n");
    return false;
}

bool roboticslab::AravisGigE::setMode(int feature, FeatureMode mode)
{
    CD_ERROR("Not implemented!\n");
    return false;
}

bool roboticslab::AravisGigE::getMode(int feature, FeatureMode *mode)
{
    CD_ERROR("Not implemented!\n");
    return false;
}

bool roboticslab::AravisGigE::setOnePush(int feature)
{
    CD_ERROR("Not implemented!\n");
    return false;
}

