#include "AravisGigE.hpp"


bool roboticslab::AravisGigE::getCameraDescription(CameraDescriptor *camera)
{
    camera->busType = BUS_UNKNOWN; //-- Temporary until we add a BUS_GIGE in YARP
    camera->deviceDescription = std::string(arv_camera_get_device_id(this->camera)) + ": "
            + arv_camera_get_model_name(this->camera);
    return true;
}

bool roboticslab::AravisGigE::hasFeature(int feature, bool *hasFeature)
{
    CD_DEBUG("Request to know if camera has feature %d\n", feature);

    //-- Check if YARP supports this feature
    cameraFeature_id_t f;
    f = static_cast<cameraFeature_id_t>(feature);
    if (f < YARP_FEATURE_BRIGHTNESS || f > YARP_FEATURE_NUMBER_OF-1)
    {
        CD_ERROR("Feature not supported by YARP\n");
        return false;
    }

    //-- Check if device supports this feature
    if (yarp_arv_int_feature_map.find(f) != yarp_arv_int_feature_map.end() ||
            yarp_arv_float_feat_map.find(f) != yarp_arv_float_feat_map.end())
    {
        *hasFeature = true;
    }
    else
    {
        *hasFeature = false;
    }

    return true;
}

bool roboticslab::AravisGigE::setFeature(int feature, double value)
{
    CD_DEBUG("Requested to set feature %d\n", feature);
    //-- Check if YARP supports this feature
    cameraFeature_id_t f;
    f = static_cast<cameraFeature_id_t>(feature);
    if (f < YARP_FEATURE_BRIGHTNESS || f > YARP_FEATURE_NUMBER_OF-1)
    {
        CD_ERROR("Feature not supported by YARP\n");
        return false;
    }

    auto yarp_int_feature = yarp_arv_int_feature_map.find(f);
    if (yarp_int_feature == yarp_arv_int_feature_map.end())
    {
        auto yarp_float_feature = yarp_arv_float_feat_map.find(f);
        if (yarp_float_feature == yarp_arv_float_feat_map.end())
        {
            CD_ERROR("Property with yarp id %d not available\n", f);
            return false;
        }

        //-- Check {here} that value is within range here (when you can inspect ranges)

        arv_device_set_float_feature_value(arv_camera_get_device(camera), yarp_float_feature->second, value);
    }
    else
    {
        //-- Check {here} that value is within range here (when you can inspect ranges)

        arv_device_set_integer_feature_value(arv_camera_get_device(camera), yarp_int_feature->second, value);
    }
    return true;
}

bool roboticslab::AravisGigE::getFeature(int feature, double *value)
{
    CD_DEBUG("Property with yarp id %d requested\n", feature);
    //-- Check if YARP supports this feature
    cameraFeature_id_t f;
    f = static_cast<cameraFeature_id_t>(feature);
    if (f < YARP_FEATURE_BRIGHTNESS || f > YARP_FEATURE_NUMBER_OF-1)
    {
        CD_ERROR("Feature not supported by YARP\n");
        return false;
    }

    auto yarp_int_feature = yarp_arv_int_feature_map.find(f);
    if (yarp_int_feature == yarp_arv_int_feature_map.end())
    {
        auto yarp_float_feature = yarp_arv_float_feat_map.find(f);
        if (yarp_float_feature == yarp_arv_float_feat_map.end())
        {
            CD_ERROR("Property with yarp id %d not available\n", f);
            return false;
        }

        *value = arv_device_get_float_feature_value(arv_camera_get_device(camera), yarp_float_feature->second);
    }
    else
    {
        *value = arv_device_get_integer_feature_value(arv_camera_get_device(camera), yarp_int_feature->second);
    }
    CD_DEBUG("Value: %f\n", *value);
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
    CD_DEBUG("Request to know if feature %d has on/off mode\n", feature);

    //-- Check if YARP supports this feature
    cameraFeature_id_t f;
    f = static_cast<cameraFeature_id_t>(feature);
    if (f < YARP_FEATURE_BRIGHTNESS || f > YARP_FEATURE_NUMBER_OF-1)
    {
        CD_ERROR("Feature not supported by YARP\n");
        return false;
    }

    //-- Check if device supports this feature
    //-- (No feature supports on/off mode currently. If any did, the code to discover that would go here)
    *HasOnOff = false;
    return true;
}

bool roboticslab::AravisGigE::setActive(int feature, bool onoff)
{
    CD_DEBUG("Requested to set on/of mode for feature %d\n", feature);
    bool b;
    if (!hasFeature(feature, &b) || !b)
    {
        return false;
    }

    if (!hasOnOff(feature, &b) || !b)
    {
        CD_ERROR("Feature does not have on/off mode. You can call hasOnOff() to know if a specific feature supports on/off mode");
        return false;
    }

    //-- Check if device supports this feature
    //-- (No feature supports on/off mode currently. If any did, the code to discover that would go here)
    return true;
}

bool roboticslab::AravisGigE::getActive(int feature, bool *isActive)
{
    CD_DEBUG("Requested to get on/of mode for feature %d\n", feature);
    bool b;
    if (!hasFeature(feature, &b) || !b)
    {
        return false;
    }

    if (!hasOnOff(feature, &b) || !b)
    {
        CD_ERROR("Feature does not have on/off mode. You can call hasOnOff() to know if a specific feature supports on/off mode");
        return false;
    }

    //-- Check if device supports this feature
    //-- (No feature supports on/off mode currently. If any did, the code to discover that would go here)
    *isActive = false;
    return true;
}

bool roboticslab::AravisGigE::hasAuto(int feature, bool *hasAuto)
{
    CD_DEBUG("Request to know if feature %d has auto mode\n", feature);

    //-- Check if YARP supports this feature
    cameraFeature_id_t f;
    f = static_cast<cameraFeature_id_t>(feature);
    if (f < YARP_FEATURE_BRIGHTNESS || f > YARP_FEATURE_NUMBER_OF-1)
    {
        CD_ERROR("Feature not supported by YARP\n");
        return false;
    }

    //-- Check if device supports this feature
    //-- (No feature supports auto mode currently. If any did, the code to discover that would go here)
    *hasAuto = false;
    return true;
}

bool roboticslab::AravisGigE::hasManual(int feature, bool *hasManual)
{
    CD_DEBUG("Request to know if feature %d has manual mode\n", feature);

    //-- Check if YARP supports this feature
    cameraFeature_id_t f;
    f = static_cast<cameraFeature_id_t>(feature);
    if (f < YARP_FEATURE_BRIGHTNESS || f > YARP_FEATURE_NUMBER_OF-1)
    {
        CD_ERROR("Feature not supported by YARP\n");
        return false;
    }

    //-- Check if device supports this feature
    //-- (No feature supports manual mode currently. If any did, the code to discover that would go here)
    *hasManual = false;
    return true;
}

bool roboticslab::AravisGigE::hasOnePush(int feature, bool *hasOnePush)
{
    CD_DEBUG("Request to know if feature %d has one push mode\n", feature);

    //-- Check if YARP supports this feature
    cameraFeature_id_t f;
    f = static_cast<cameraFeature_id_t>(feature);
    if (f < YARP_FEATURE_BRIGHTNESS || f > YARP_FEATURE_NUMBER_OF-1)
    {
        CD_ERROR("Feature not supported by YARP\n");
        return false;
    }

    //-- Check if device supports this feature
    //-- (No feature supports one push mode currently. If any did, the code to discover that would go here)
    *hasOnePush = false;
    return true;
}

bool roboticslab::AravisGigE::setMode(int feature, FeatureMode mode)
{
    CD_DEBUG("Requested to set auto/manual mode for feature %d\n", feature);
    bool b;
    if (!hasFeature(feature, &b) || !b)
    {
        return false;
    }

    if ((!hasAuto(feature, &b) || !b) && (!hasManual(feature, &b) || !b))
    {
        CD_ERROR("Feature does not have auto/manual mode. You can call hasAuto()/hasManual() to know if a specific feature supports auto/manual mode");
        return false;
    }

    //-- Check if device supports this feature
    //-- (No feature supports auto/manual mode currently. If any did, the code to discover that would go here)
    return true;
}

bool roboticslab::AravisGigE::getMode(int feature, FeatureMode *mode)
{
    CD_DEBUG("Requested to get auto/manual mode for feature %d\n", feature);
    bool b;
    if (!hasFeature(feature, &b) || !b)
    {
        return false;
    }

    if ((!hasAuto(feature, &b) || !b) && (!hasManual(feature, &b) || !b))
    {
        CD_ERROR("Feature does not have auto/manual mode. You can call hasAuto()/hasManual() to know if a specific feature supports auto/manual mode");
        return false;
    }

    //-- Check if device supports this feature
    //-- (No feature supports auto/manual mode currently. If any did, the code to discover that would go here)
    *mode = MODE_UNKNOWN;
    return true;
}

bool roboticslab::AravisGigE::setOnePush(int feature)
{
    CD_DEBUG("Requested to set one push mode for feature %d\n", feature);
    bool b;
    if (!hasFeature(feature, &b) || !b)
    {
        return false;
    }

    if (!hasOnePush(feature, &b) || !b)
    {
        CD_ERROR("Feature does not have one push mode. You can call hasOnePush() to know if a specific feature supports one push mode");
        return false;
    }

    //-- Check if device supports this feature
    //-- (No feature supports one push mode currently. If any did, the code to discover that would go here)
    return true;
}

