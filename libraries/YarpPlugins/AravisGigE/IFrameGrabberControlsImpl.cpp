#include "AravisGigE.hpp"

#include <yarp/os/LogStream.h>

#include "LogComponent.hpp"

using namespace roboticslab;

bool AravisGigE::getCameraDescription(CameraDescriptor *camera)
{
    camera->busType = BUS_UNKNOWN; //-- Temporary until we add a BUS_GIGE in YARP
    camera->deviceDescription = std::string(arv_camera_get_device_id(this->camera)) + ": "
            + arv_camera_get_model_name(this->camera);
    return true;
}

bool AravisGigE::hasFeature(int feature, bool *hasFeature)
{
    yCDebug(ARV) << "Request to know if camera has feature" << feature;

    //-- Check if YARP supports this feature
    cameraFeature_id_t f;
    f = static_cast<cameraFeature_id_t>(feature);
    if (f < YARP_FEATURE_BRIGHTNESS || f > YARP_FEATURE_NUMBER_OF-1)
    {
        yCError(ARV) << "Feature not supported by YARP";
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

bool AravisGigE::setFeature(int feature, double value)
{
    yCDebug(ARV) << "Requested to set feature" << feature;
    //-- Check if YARP supports this feature
    cameraFeature_id_t f;
    f = static_cast<cameraFeature_id_t>(feature);
    if (f < YARP_FEATURE_BRIGHTNESS || f > YARP_FEATURE_NUMBER_OF-1)
    {
        yCError(ARV) << "Feature not supported by YARP";
        return false;
    }

    std::map<cameraFeature_id_t, const char*>::iterator yarp_int_feature = yarp_arv_int_feature_map.find(f);
    if (yarp_int_feature == yarp_arv_int_feature_map.end())
    {
        std::map<cameraFeature_id_t, const char*>::iterator yarp_float_feature = yarp_arv_float_feat_map.find(f);
        if (yarp_float_feature == yarp_arv_float_feat_map.end())
        {
            yCError(ARV) << "Property with yarp id" << f << "not available";
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

bool AravisGigE::getFeature(int feature, double *value)
{
    yCDebug(ARV) << "Property with yarp id" << feature << "requested";
    //-- Check if YARP supports this feature
    cameraFeature_id_t f;
    f = static_cast<cameraFeature_id_t>(feature);
    if (f < YARP_FEATURE_BRIGHTNESS || f > YARP_FEATURE_NUMBER_OF-1)
    {
        yCError(ARV) << "Feature not supported by YARP";
        return false;
    }

    std::map<cameraFeature_id_t, const char*>::iterator yarp_int_feature = yarp_arv_int_feature_map.find(f);
    if (yarp_int_feature == yarp_arv_int_feature_map.end())
    {
        std::map<cameraFeature_id_t, const char*>::iterator yarp_float_feature = yarp_arv_float_feat_map.find(f);
        if (yarp_float_feature == yarp_arv_float_feat_map.end())
        {
            yCError(ARV) << "Property with yarp id" << f << "not available";
            return false;
        }

        *value = arv_device_get_float_feature_value(arv_camera_get_device(camera), yarp_float_feature->second);
    }
    else
    {
        *value = arv_device_get_integer_feature_value(arv_camera_get_device(camera), yarp_int_feature->second);
    }
    yCDebug(ARV) << "Value:" << *value;
    return true;
}

bool AravisGigE::setFeature(int feature, double value1, double value2)
{
    yCError(ARV) << "No features with 2 values supported!";
    return false;
}

bool AravisGigE::getFeature(int feature, double *value1, double *value2)
{
    yCError(ARV) << "No features with 2 values supported!";
    return false;
}

bool AravisGigE::hasOnOff(int feature, bool *HasOnOff)
{
    yCDebug(ARV) << "Request to know if feature" << feature << "has on/off mode";

    //-- Check if YARP supports this feature
    cameraFeature_id_t f;
    f = static_cast<cameraFeature_id_t>(feature);
    if (f < YARP_FEATURE_BRIGHTNESS || f > YARP_FEATURE_NUMBER_OF-1)
    {
        yCError(ARV) << "Feature not supported by YARP";
        return false;
    }

    //-- Check if device supports this feature
    //-- (No feature supports on/off mode currently. If any did, the code to discover that would go here)
    *HasOnOff = false;
    return true;
}

bool AravisGigE::setActive(int feature, bool onoff)
{
    yCDebug(ARV) << "Requested to set on/of mode for feature" << feature;
    bool b;
    if (!hasFeature(feature, &b) || !b)
    {
        return false;
    }

    if (!hasOnOff(feature, &b) || !b)
    {
        yCError(ARV) << "Feature does not have on/off mode; you can call hasOnOff() to know if a specific feature supports on/off mode";
        return false;
    }

    //-- Check if device supports this feature
    //-- (No feature supports on/off mode currently. If any did, the code to discover that would go here)
    return true;
}

bool AravisGigE::getActive(int feature, bool *isActive)
{
    yCDebug(ARV) << "Requested to get on/of mode for feature" << feature;
    bool b;
    if (!hasFeature(feature, &b) || !b)
    {
        return false;
    }

    if (!hasOnOff(feature, &b) || !b)
    {
        yCError(ARV) << "Feature does not have on/off mode; you can call hasOnOff() to know if a specific feature supports on/off mode";
        return false;
    }

    //-- Check if device supports this feature
    //-- (No feature supports on/off mode currently. If any did, the code to discover that would go here)
    *isActive = false;
    return true;
}

bool AravisGigE::hasAuto(int feature, bool *hasAuto)
{
    yCDebug(ARV) << "Request to know if feature" << feature << "has auto mode";

    //-- Check if YARP supports this feature
    cameraFeature_id_t f;
    f = static_cast<cameraFeature_id_t>(feature);
    if (f < YARP_FEATURE_BRIGHTNESS || f > YARP_FEATURE_NUMBER_OF-1)
    {
        yCError(ARV) << "Feature not supported by YARP";
        return false;
    }

    //-- Check if device supports this feature
    //-- (No feature supports auto mode currently. If any did, the code to discover that would go here)
    *hasAuto = false;
    return true;
}

bool AravisGigE::hasManual(int feature, bool *hasManual)
{
    yCDebug(ARV) << "Request to know if feature" << feature << "has manual mode";

    //-- Check if YARP supports this feature
    cameraFeature_id_t f;
    f = static_cast<cameraFeature_id_t>(feature);
    if (f < YARP_FEATURE_BRIGHTNESS || f > YARP_FEATURE_NUMBER_OF-1)
    {
        yCError(ARV) << "Feature not supported by YARP";
        return false;
    }

    //-- Check if device supports this feature
    //-- (No feature supports manual mode currently. If any did, the code to discover that would go here)
    *hasManual = false;
    return true;
}

bool AravisGigE::hasOnePush(int feature, bool *hasOnePush)
{
    yCDebug(ARV) << "Request to know if feature" << feature << "has one push mode";

    //-- Check if YARP supports this feature
    cameraFeature_id_t f;
    f = static_cast<cameraFeature_id_t>(feature);
    if (f < YARP_FEATURE_BRIGHTNESS || f > YARP_FEATURE_NUMBER_OF-1)
    {
        yCError(ARV) << "Feature not supported by YARP";
        return false;
    }

    //-- Check if device supports this feature
    //-- (No feature supports one push mode currently. If any did, the code to discover that would go here)
    *hasOnePush = false;
    return true;
}

bool AravisGigE::setMode(int feature, FeatureMode mode)
{
    yCDebug(ARV) << "Requested to set auto/manual mode for feature" << feature;
    bool b;
    if (!hasFeature(feature, &b) || !b)
    {
        return false;
    }

    if ((!hasAuto(feature, &b) || !b) && (!hasManual(feature, &b) || !b))
    {
        yCError(ARV) << "Feature does not have auto/manual mode; you can call hasAuto()/hasManual() to know if a specific feature supports auto/manual mode";
        return false;
    }

    //-- Check if device supports this feature
    //-- (No feature supports auto/manual mode currently. If any did, the code to discover that would go here)
    return true;
}

bool AravisGigE::getMode(int feature, FeatureMode *mode)
{
    yCDebug(ARV) << "Requested to get auto/manual mode for feature" << feature;
    bool b;
    if (!hasFeature(feature, &b) || !b)
    {
        return false;
    }

    if ((!hasAuto(feature, &b) || !b) && (!hasManual(feature, &b) || !b))
    {
        yCError(ARV) << "Feature does not have auto/manual mode; you can call hasAuto()/hasManual() to know if a specific feature supports auto/manual mode";
        return false;
    }

    //-- Check if device supports this feature
    //-- (No feature supports auto/manual mode currently. If any did, the code to discover that would go here)
    *mode = MODE_UNKNOWN;
    return true;
}

bool AravisGigE::setOnePush(int feature)
{
    yCDebug(ARV) << "Requested to set one push mode for feature" << feature;
    bool b;
    if (!hasFeature(feature, &b) || !b)
    {
        return false;
    }

    if (!hasOnePush(feature, &b) || !b)
    {
        yCError(ARV) << "Feature does not have one push mode; you can call hasOnePush() to know if a specific feature supports one push mode";
        return false;
    }

    //-- Check if device supports this feature
    //-- (No feature supports one push mode currently. If any did, the code to discover that would go here)
    return true;
}
