#include "AravisGigE.hpp"

bool roboticslab::AravisGigE::setZoom(int v)
{
    if (zoomMin<=v && v>=zoomMax)
    {
        arv_device_set_integer_feature_value(arv_camera_get_device(camera), "Zoom", v);
        int zoom = getZoom();
        if (zoom!=v)
        {
            CD_ERROR("Some error ocurred when trying to set property \"Zoom\"\n");
            return false;
        }
        return true;
    }
    else
    {
        CD_ERROR("Value %d for property \"Zoom\" out of range (%d, %d)\n", v, zoomMin, zoomMax);
        return false;
    }
}

bool roboticslab::AravisGigE::setFocus(int v)
{
    if (focusMin<=v && v>=focusMax)
    {
        arv_device_set_integer_feature_value(arv_camera_get_device(camera), "Focus", v);
        int focus = getFocus();
        if (focus!=v)
        {
            CD_ERROR("Some error ocurred when trying to set property \"Focus\"\n");
            return false;
        }
        return true;
    }
    else
    {
        CD_ERROR("Value %d for property \"Focus\" out of range (%d, %d)\n", v, focusMin, focusMax);
        return false;
    }
}

//bool roboticslab::AravisGigE::setIris(int v)
//{

//}

int roboticslab::AravisGigE::getZoom()
{
    if (zoomAvailable)
    {
        return arv_device_get_integer_feature_value(arv_camera_get_device(camera), "Zoom");
    }
    else
    {
        CD_ERROR("Property \"Zoom\" not available\n");
        return -1;
    }
}

int roboticslab::AravisGigE::getFocus()
{
    if (focusAvailable)
    {
        return arv_device_get_integer_feature_value(arv_camera_get_device(camera), "Focus");
    }
    else
    {
        CD_ERROR("Property \"Focus\" not available\n");
        return -1;
    }
}

//int roboticslab::AravisGigE::getIris()
//{

//}
