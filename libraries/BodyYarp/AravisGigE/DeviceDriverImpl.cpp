#include "AravisGigE.hpp"

bool roboticslab::AravisGigE::open(yarp::os::Searchable &config)
{
    CD_INFO("Opening AravisGigE device\n");
    arv_enable_interface ("Fake"); //-- Enables fake Aravis cameras (useful for debug / testing)

    //-- Data initialization
    //-------------------------------------------------------------------------------
    camera = NULL;
    stream = NULL;
    framebuffer = NULL;

    payload = 0;

    widthMin = widthMax = heightMin = heightMax = 0;
    xoffset = yoffset = _width = _height = 0;
    fpsMin = fpsMax = gainMin = gainMax = exposureMin = exposureMax = 0;
    controlExposure = false;
    targetGrey = 0;
    frameID = prevFrameID = 0;
    num_buffers = 50;

    //-- Open Aravis device(s)
    //-------------------------------------------------------------------------------
    int index = 0; //-- Right now, index is hardcoded (in the future could be a param)

    //-- Get device name (from index)
    std::string deviceName;

    arv_update_device_list();
    if ((index<0) || (index>=(int)arv_get_n_devices()))
    {
        CD_ERROR("Invalid device index. Device index should be 0<index<num_devices.\n");
        return false;
    }
    deviceName = arv_get_device_id(index);

    //-- Create Aravis camera
    camera = arv_camera_new(deviceName.c_str());
    if (camera != NULL)
    {
        CD_INFO("Created Aravis camera with index %d, named \"%s\"\n", index, deviceName.c_str());
    }
    else
    {
        CD_ERROR("Could not create Aravis camera.\n");
        return false;
    }

    //-- Once we have a camera, we obtain the camera properties limits
    pixelFormats = arv_camera_get_available_pixel_formats(camera, &pixelFormatsCnt);

    arv_camera_get_width_bounds(camera, &widthMin, &widthMax);
    arv_camera_get_height_bounds(camera, &heightMin, &heightMax);
    arv_camera_set_region(camera, 0, 0, widthMax, heightMax);
    CD_INFO("Width range: min=%d max=%d\n", widthMin, widthMax);
    CD_INFO("Height range: min=%d max=%d\n", heightMin, heightMax);

    fpsAvailable = arv_camera_is_frame_rate_available(camera);
    if (fpsAvailable)
    {
        arv_camera_get_frame_rate_bounds(camera, &fpsMin, &fpsMax);
        CD_INFO("FPS range: min=%d max=%d\n", fpsMin, fpsMax);
    }
    else
        CD_WARNING("FPS property not available\n");

    gainAvailable = arv_camera_is_gain_available(camera);
    if (gainAvailable)
    {
        arv_camera_get_gain_bounds (camera, &gainMin, &gainMax);
        CD_INFO("Gain range: min=%d max=%d\n", gainMin, gainMax);
    }
    else
        CD_WARNING("Gain property not available\n");

    exposureAvailable = arv_camera_is_exposure_time_available(camera);
    if (exposureAvailable)
    {
        arv_camera_get_exposure_time_bounds (camera, &exposureMin, &exposureMax);
        CD_INFO("Exposure range: min=%d max=%d\n", exposureMin, exposureMax);
    }
    else
        CD_WARNING("Gain property not available\n");


    return true;
}

bool roboticslab::AravisGigE::close()
{
    CD_INFO("AravisGigE driver is closed!\n");
    return true;
}
