#include "AravisGigE.hpp"

#include <yarp/os/LogStream.h>

#include "LogComponent.hpp"

bool AravisGigE::open(yarp::os::Searchable &config)
{
    //-- Configuration of Aravis GigE Camera device
    if (config.check("fake", "enable fake Aravis camera"))
    {
        yCInfo(ARV) << "Enabling fake Aravis camera";
        arv_enable_interface("Fake"); //-- Enables fake Aravis cameras (useful for debug / testing)
    }

    //-- Data initialization
    //-------------------------------------------------------------------------------
    camera = nullptr;
    stream = nullptr;
    framebuffer = nullptr;

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
        yCError(ARV) << "Invalid device index, should be 0 <" << index << "< num_devices";
        return false;
    }
    deviceName = arv_get_device_id(index);

    //-- Create Aravis camera
    camera = arv_camera_new(deviceName.c_str());
    if (camera != nullptr)
    {
        yCInfo(ARV) << "Created Aravis camera with index" << index << "and name" << deviceName;
    }
    else
    {
        yCError(ARV) << "Could not create Aravis camera";
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
        yCInfo(ARV) << "Available pixel formats:";
        for (int i = 0; i < n_pixel_formats; i++)
             yCInfo(ARV) << available_formats[i];
    }
    yCInfo(ARV) << "Pixel format selected:" << arv_camera_get_pixel_format_as_string(camera);

    arv_camera_get_width_bounds(camera, &widthMin, &widthMax);
    arv_camera_get_height_bounds(camera, &heightMin, &heightMax);
    arv_camera_set_region(camera, 0, 0, widthMax, heightMax);
    yCInfo(ARV, "Width range: min=%d max=%d", widthMin, widthMax);
    yCInfo(ARV, "Height range: min=%d max=%d", heightMin, heightMax);

    fpsAvailable = arv_camera_is_frame_rate_available(camera);
    if (fpsAvailable)
    {
        arv_camera_get_frame_rate_bounds(camera, &fpsMin, &fpsMax);
        yCInfo(ARV, "FPS range: min=%f max=%f", fpsMin, fpsMax);

        fps = arv_camera_get_frame_rate(camera);
        yCInfo(ARV) << "Current FPS value:" << fps;
    }
    else
        yCWarning(ARV) << "FPS property not available";


    gainAvailable = arv_camera_is_gain_available(camera);
    if (gainAvailable)
    {
        arv_camera_get_gain_bounds (camera, &gainMin, &gainMax);
        yCInfo(ARV, "Gain range: min=%f max=%f", gainMin, gainMax);

        gain = arv_camera_get_gain(camera);
        yCInfo(ARV) << "Current gain value:" << gain;
    }
    else
        yCWarning(ARV) << "Gain property not available";

    exposureAvailable = arv_camera_is_exposure_time_available(camera);
    if (exposureAvailable)
    {
        arv_camera_get_exposure_time_bounds (camera, &exposureMin, &exposureMax);
        yCInfo(ARV, "Exposure range: min=%f max=%f", exposureMin, exposureMax);

        exposure = arv_camera_get_exposure_time(camera);
        yCInfo(ARV) << "Current exposure value:" << exposure;
    }
    else
        yCWarning(ARV) << "Gain property not available";

    //-- Lens controls availability
    yCInfo(ARV) << "Checking Lens Controls availability";
    arv_device_get_feature(arv_camera_get_device(camera), "Zoom") == nullptr ? zoomAvailable = false : zoomAvailable = true;
    if (zoomAvailable)
    {
        arv_device_get_integer_feature_bounds(arv_camera_get_device(camera), "Zoom", &zoomMin, &zoomMax);
        yCInfo(ARV, "Zoom range: min=%ld max=%ld", zoomMin, zoomMax);
    }
    else
        yCWarning(ARV) << "Zoom property not available";

    arv_device_get_feature(arv_camera_get_device(camera), "Focus") == nullptr ? focusAvailable = false : focusAvailable= true;
    if (focusAvailable)
    {
        arv_device_get_integer_feature_bounds(arv_camera_get_device(camera), "Focus", &focusMin, &focusMax);
        yCInfo(ARV, "Focus range: min=%ld max=%ld", focusMin, focusMax);
    }
    else
        yCWarning(ARV) << "Focus property not available";

    //-- Start capturing images
    //-------------------------------------------------------------------------------

    //-- Initialization of buffer(s)
    if (stream)
    {
        g_object_unref(stream);
        stream = nullptr;
    }
    stream = arv_camera_create_stream(camera, nullptr, nullptr);
    if (stream == nullptr)
    {
        yCError(ARV) << "Could not create Aravis stream";
        return false;
    }
    g_object_set(stream, "socket-buffer", ARV_GV_STREAM_SOCKET_BUFFER_AUTO, "socket-buffer-size", 0, nullptr);
    g_object_set(stream, "packet-resend", ARV_GV_STREAM_PACKET_RESEND_NEVER, nullptr);
    g_object_set(stream, "packet-timeout", (unsigned) 40000, "frame-retention", (unsigned) 200000, nullptr);

    payload = arv_camera_get_payload (camera);

    for (int i = 0; i < num_buffers; i++)
        arv_stream_push_buffer(stream, arv_buffer_new(payload, nullptr));

    //-- Start continuous acquisition
    arv_camera_set_acquisition_mode(camera, ARV_ACQUISITION_MODE_CONTINUOUS);
    arv_device_set_string_feature_value(arv_camera_get_device(camera), "TriggerMode" , "Off");
    arv_camera_start_acquisition(camera);
    yCInfo(ARV) << "Aravis Camera acquisition started!";
    return true;
}

bool AravisGigE::close()
{
    if(camera==nullptr)
    {
        yCError(ARV) << "Camera was not started!";
        return false;
    }

    arv_camera_stop_acquisition(camera);
    yCInfo(ARV) << "Aravis Camera acquisition stopped!";

    //-- Cleanup
    if(stream)
    {
        g_object_unref(stream);
        stream = nullptr;
    }

    g_object_unref(camera);
    camera = nullptr;

    return true;
}
