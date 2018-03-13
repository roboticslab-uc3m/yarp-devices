#include "AravisGigE.hpp"

bool roboticslab::AravisGigE::open(yarp::os::Searchable &config)
{
    CD_INFO("Opening AravisGigE device\n");

    //-- Configuration of Aravis GigE Camera device
    if (config.check("fake", "enable fake Aravis camera"))
    {
        CD_INFO("Enabling fake Aravis camera\n");
        arv_enable_interface("Fake"); //-- Enables fake Aravis cameras (useful for debug / testing)
    }

    //-- Data initialization
    //-------------------------------------------------------------------------------
    camera = NULL;
    stream = NULL;
    framebuffer = NULL;

    payload = 0;

    widthMin = widthMax = heightMin = heightMax = 0;
    xoffset = yoffset = _width = _height = 0;
    fpsMin = fpsMax = fps = 0;
    gainMin = gainMax = gain = 0;
    exposureMin = exposureMax = exposure = 0;
    controlExposure = false;
    targetGrey = 0;
    frameID = prevFrameID = 0;
    num_buffers = 50;

    //-- Initalize parameter mapping
    yarp_arv_float_feat_map[YARP_FEATURE_EXPOSURE]  ="ExposureTime";
    yarp_arv_float_feat_map[YARP_FEATURE_GAIN] = "Gain";
    yarp_arv_float_feat_map[YARP_FEATURE_FRAME_RATE] = "FPS";
    yarp_arv_int_feature_map[YARP_FEATURE_ZOOM] = "Zoom";
    yarp_arv_int_feature_map[YARP_FEATURE_FOCUS] = "Focus";


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

    //-- Once we have a camera, we obtain the camera properties limits and initial values
    pixelFormats = arv_camera_get_available_pixel_formats(camera, &pixelFormatsCnt);
    pixelFormat = arv_camera_get_pixel_format(camera);
    if (config.check("introspection", "print available pixel formats on device init"))
    {
        //-- List all  available formats
        guint n_pixel_formats;
        const char ** available_formats = arv_camera_get_available_pixel_formats_as_display_names(camera, &n_pixel_formats);
        CD_INFO("Available pixel formats:\n");
        for (int i = 0; i < n_pixel_formats; i++)
             CD_INFO("\t- %s\n", available_formats[i]);
    }
    CD_INFO("Pixel format selected: %s\n", arv_camera_get_pixel_format_as_string(camera));

    arv_camera_get_width_bounds(camera, &widthMin, &widthMax);
    arv_camera_get_height_bounds(camera, &heightMin, &heightMax);
    arv_camera_set_region(camera, 0, 0, widthMax, heightMax);
    CD_INFO("Width range: min=%d max=%d\n", widthMin, widthMax);
    CD_INFO("Height range: min=%d max=%d\n", heightMin, heightMax);

    fpsAvailable = arv_camera_is_frame_rate_available(camera);
    if (fpsAvailable)
    {
        arv_camera_get_frame_rate_bounds(camera, &fpsMin, &fpsMax);
        CD_INFO("FPS range: min=%f max=%f\n", fpsMin, fpsMax);

        fps = arv_camera_get_frame_rate(camera);
        CD_INFO("Current FPS value: %f\n", fps);
    }
    else
        CD_WARNING("FPS property not available\n");


    gainAvailable = arv_camera_is_gain_available(camera);
    if (gainAvailable)
    {
        arv_camera_get_gain_bounds (camera, &gainMin, &gainMax);
        CD_INFO("Gain range: min=%f max=%f\n", gainMin, gainMax);

        gain = arv_camera_get_gain(camera);
        CD_INFO("Current gain value: %f\n", gain);
    }
    else
        CD_WARNING("Gain property not available\n");

    exposureAvailable = arv_camera_is_exposure_time_available(camera);
    if (exposureAvailable)
    {
        arv_camera_get_exposure_time_bounds (camera, &exposureMin, &exposureMax);
        CD_INFO("Exposure range: min=%f max=%f\n", exposureMin, exposureMax);

        exposure = arv_camera_get_exposure_time(camera);
        CD_INFO("Current exposure value: %f\n", exposure);
    }
    else
        CD_WARNING("Gain property not available\n");

    //-- Lens controls availability
    CD_INFO("Lens Controls available:\n");
    arv_device_get_feature(arv_camera_get_device(camera), "Zoom") == NULL ? zoomAvailable = false : zoomAvailable = true;
    if (zoomAvailable)
    {
        arv_device_get_integer_feature_bounds(arv_camera_get_device(camera), "Zoom", &zoomMin, &zoomMax);
        CD_INFO("\tZoom range: min=%d max=%d\n", zoomMin, zoomMax);
    }
    else
        CD_WARNING("\tZoom property not available\n");

    arv_device_get_feature(arv_camera_get_device(camera), "Focus") == NULL ? focusAvailable = false : focusAvailable= true;
    if (focusAvailable)
    {
        arv_device_get_integer_feature_bounds(arv_camera_get_device(camera), "Focus", &focusMin, &focusMax);
        CD_INFO("\tFocus range: min=%d max=%d\n", focusMin, focusMax);
    }
    else
        CD_WARNING("\tFocus property not available\n");

    //-- Start capturing images
    //-------------------------------------------------------------------------------

    //-- Initialization of buffer(s)
    if (stream)
    {
        g_object_unref(stream);
        stream = NULL;
    }
    stream = arv_camera_create_stream(camera, NULL, NULL);
    if (stream == NULL)
    {
        CD_ERROR("Could not create Aravis stream.\n");
        return false;
    }
    g_object_set(stream, "socket-buffer", ARV_GV_STREAM_SOCKET_BUFFER_AUTO, "socket-buffer-size", 0, NULL);
    g_object_set(stream, "packet-resend", ARV_GV_STREAM_PACKET_RESEND_NEVER, NULL);
    g_object_set(stream, "packet-timeout", (unsigned) 40000, "frame-retention", (unsigned) 200000, NULL);

    payload = arv_camera_get_payload (camera);

    for (int i = 0; i < num_buffers; i++)
        arv_stream_push_buffer(stream, arv_buffer_new(payload, NULL));

    //-- Start continuous acquisition
    arv_camera_set_acquisition_mode(camera, ARV_ACQUISITION_MODE_CONTINUOUS);
    arv_device_set_string_feature_value(arv_camera_get_device(camera), "TriggerMode" , "Off");
    arv_camera_start_acquisition(camera);
    CD_INFO("Aravis Camera acquisition started!\n");
    return true;
}

bool roboticslab::AravisGigE::close()
{
    if(camera==NULL)
    {
        CD_ERROR("Camera was not started!\n");
        return false;
    }

    arv_camera_stop_acquisition(camera);
    CD_INFO("Aravis Camera acquisition stopped!\n");

    //-- Cleanup
    if(stream)
    {
        g_object_unref(stream);
        stream = NULL;
    }

    g_object_unref(camera);
    camera = NULL;

    return true;
}
