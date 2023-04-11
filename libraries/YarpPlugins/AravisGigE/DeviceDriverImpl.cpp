#include "AravisGigE.hpp"

#include <string>
#include <unordered_set>

#include <yarp/os/LogStream.h>

#include "LogComponent.hpp"

using namespace roboticslab;

bool AravisGigE::open(yarp::os::Searchable &config)
{
    //-- Configuration of Aravis GigE Camera device
    if (config.check("fake", "enable fake Aravis camera"))
    {
        yCInfo(ARV) << "Enabling fake Aravis camera";
        arv_enable_interface("Fake"); //-- Enables fake Aravis cameras (useful for debug / testing)
    }

    //-- Open Aravis device(s)
    //-------------------------------------------------------------------------------
    int index = config.check("index", yarp::os::Value(0), "camera index").asInt32();

    arv_update_device_list();

    if (index < 0 || index >= (int)arv_get_n_devices())
    {
        yCError(ARV) << "Invalid device index, should be 0 <" << index << "<" << arv_get_n_devices();
        return false;
    }

    const auto * deviceName = arv_get_device_id(index);

    //-- Create Aravis camera
#if ARAVIS_CHECK_VERSION(0, 7, 3)
    camera = arv_camera_new(deviceName, nullptr);
#else
    camera = arv_camera_new(deviceName);
#endif

    if (camera != nullptr)
    {
        yCInfo(ARV) << "Created Aravis camera with index" << index << "and name" << deviceName;
    }
    else
    {
        yCError(ARV) << "Could not create Aravis camera with index" << index << "and name" << deviceName;
        return false;
    }

    //-- Once we have a camera, we obtain the camera properties limits and initial values
    std::unordered_set<std::string> availablePixelFormats;

    if (config.check("introspection", "print available pixel formats on device init"))
    {
        //-- List all  available formats
        guint n_pixel_formats;
#if ARAVIS_CHECK_VERSION(0, 7, 4)
        auto ** availableFormatsStrings = arv_camera_dup_available_pixel_formats_as_strings(camera, &n_pixel_formats, nullptr);
        auto ** availableFormatsNames = arv_camera_dup_available_pixel_formats_as_display_names(camera, &n_pixel_formats, nullptr);
#elif ARAVIS_CHECK_VERSION(0, 7, 3)
        auto ** availableFormatsStrings = arv_camera_get_available_pixel_formats_as_strings(camera, &n_pixel_formats, nullptr);
        auto ** availableFormatsNames = arv_camera_get_available_pixel_formats_as_display_names(camera, &n_pixel_formats, nullptr);
#else
        auto ** availableFormatsStrings = arv_camera_get_available_pixel_formats_as_strings(camera, &n_pixel_formats);
        auto ** availableFormatsNames = arv_camera_get_available_pixel_formats_as_display_names(camera, &n_pixel_formats);
#endif
        yCInfo(ARV) << "Available pixel formats:";

        for (int i = 0; i < n_pixel_formats; i++)
        {
            yCInfo(ARV, "- %s (setting: %s)", availableFormatsNames[i], availableFormatsStrings[i]);
            availablePixelFormats.emplace(availableFormatsStrings[i]);
        }

        g_free(availableFormatsStrings);
        g_free(availableFormatsNames);
    }

    if (config.check("pixelFormat", "pixel format"))
    {
        //-- Set pixel format
        auto requestedPixelFormatString = config.find("pixelFormat").asString();

        if (availablePixelFormats.find(requestedPixelFormatString) == availablePixelFormats.end())
        {
            yCError(ARV) << "Requested pixel format" << requestedPixelFormatString << "is not available";
            return false;
        }

        yCInfo(ARV) << "Setting pixel format to" << requestedPixelFormatString;

#if ARAVIS_CHECK_VERSION(0, 7, 3)
        arv_camera_set_pixel_format_from_string(camera, requestedPixelFormatString.c_str(), nullptr);
#else
        arv_camera_set_pixel_format_from_string(camera, requestedPixelFormatString.c_str());
#endif
    }
    else
    {
#if ARAVIS_CHECK_VERSION(0, 7, 3)
    yCInfo(ARV) << "Using pixel format:" << arv_camera_get_pixel_format_as_string(camera, nullptr);
#else
    yCInfo(ARV) << "Using pixel format:" << arv_camera_get_pixel_format_as_string(camera);
#endif
    }

#if ARAVIS_CHECK_VERSION(0, 7, 3)
    pixelFormat = arv_camera_get_pixel_format(camera, nullptr);
#else
    pixelFormat = arv_camera_get_pixel_format(camera);
#endif

#if ARAVIS_CHECK_VERSION(0, 7, 3)
    arv_camera_get_width_bounds(camera, &widthMin, &widthMax, nullptr);
    arv_camera_get_height_bounds(camera, &heightMin, &heightMax, nullptr);
    arv_camera_set_region(camera, 0, 0, widthMax, heightMax, nullptr);
#else
    arv_camera_get_width_bounds(camera, &widthMin, &widthMax);
    arv_camera_get_height_bounds(camera, &heightMin, &heightMax);
    arv_camera_set_region(camera, 0, 0, widthMax, heightMax);
#endif

    yCInfo(ARV, "Width range: min=%d max=%d", widthMin, widthMax);
    yCInfo(ARV, "Height range: min=%d max=%d", heightMin, heightMax);

#if ARAVIS_CHECK_VERSION(0, 7, 3)
    bool fpsAvailable = arv_camera_is_frame_rate_available(camera, nullptr);
#else
    bool fpsAvailable = arv_camera_is_frame_rate_available(camera);
#endif

    if (fpsAvailable)
    {
#if ARAVIS_CHECK_VERSION(0, 7, 3)
        arv_camera_get_frame_rate_bounds(camera, &fpsMin, &fpsMax, nullptr);
#else
        arv_camera_get_frame_rate_bounds(camera, &fpsMin, &fpsMax);
#endif
        yCInfo(ARV, "FPS range: min=%f max=%f", fpsMin, fpsMax);

#if ARAVIS_CHECK_VERSION(0, 7, 3)
        fps = arv_camera_get_frame_rate(camera, nullptr);
#else
        fps = arv_camera_get_frame_rate(camera);
#endif
        yCInfo(ARV) << "Current FPS value:" << fps;
    }
    else
    {
        yCWarning(ARV) << "FPS property not available";
    }

#if ARAVIS_CHECK_VERSION(0, 7, 3)
    bool gainAvailable = arv_camera_is_gain_available(camera, nullptr);
#else
    bool gainAvailable = arv_camera_is_gain_available(camera);
#endif

    if (gainAvailable)
    {
#if ARAVIS_CHECK_VERSION(0, 7, 3)
        arv_camera_get_gain_bounds(camera, &gainMin, &gainMax, nullptr);
#else
        arv_camera_get_gain_bounds (camera, &gainMin, &gainMax);
#endif
        yCInfo(ARV, "Gain range: min=%f max=%f", gainMin, gainMax);

#if ARAVIS_CHECK_VERSION(0, 7, 3)
        gain = arv_camera_get_gain(camera, nullptr);
#else
        gain = arv_camera_get_gain(camera);
#endif
        yCInfo(ARV) << "Current gain value:" << gain;
    }
    else
    {
        yCWarning(ARV) << "Gain property not available";
    }

#if ARAVIS_CHECK_VERSION(0, 7, 3)
    bool exposureAvailable = arv_camera_is_exposure_time_available(camera, nullptr);
#else
    bool exposureAvailable = arv_camera_is_exposure_time_available(camera);
#endif

    if (exposureAvailable)
    {
#if ARAVIS_CHECK_VERSION(0, 7, 3)
        arv_camera_get_exposure_time_bounds(camera, &exposureMin, &exposureMax, nullptr);
#else
        arv_camera_get_exposure_time_bounds (camera, &exposureMin, &exposureMax);
#endif
        yCInfo(ARV, "Exposure range: min=%f max=%f", exposureMin, exposureMax);

#if ARAVIS_CHECK_VERSION(0, 7, 3)
        exposure = arv_camera_get_exposure_time(camera, nullptr);
#else
        exposure = arv_camera_get_exposure_time(camera);
#endif
        yCInfo(ARV) << "Current exposure value:" << exposure;
    }
    else
    {
        yCWarning(ARV) << "Gain property not available";
    }

    //-- Lens controls availability
    yCInfo(ARV) << "Checking Lens Controls availability";

    if (gint64 zoomMin, zoomMax; arv_device_get_feature(arv_camera_get_device(camera), "Zoom") != nullptr)
    {
#if ARAVIS_CHECK_VERSION(0, 7, 3)
        arv_device_get_integer_feature_bounds(arv_camera_get_device(camera), "Zoom", &zoomMin, &zoomMax, nullptr);
#else
        arv_device_get_integer_feature_bounds(arv_camera_get_device(camera), "Zoom", &zoomMin, &zoomMax);
#endif
        yCInfo(ARV, "Zoom range: min=%ld max=%ld", zoomMin, zoomMax);
    }
    else
    {
        yCWarning(ARV) << "Zoom property not available";
    }

    if (gint64 focusMin, focusMax; arv_device_get_feature(arv_camera_get_device(camera), "Focus") != nullptr)
    {
#if ARAVIS_CHECK_VERSION(0, 7, 3)
        arv_device_get_integer_feature_bounds(arv_camera_get_device(camera), "Focus", &focusMin, &focusMax, nullptr);
#else
        arv_device_get_integer_feature_bounds(arv_camera_get_device(camera), "Focus", &focusMin, &focusMax);
#endif
        yCInfo(ARV, "Focus range: min=%ld max=%ld", focusMin, focusMax);
    }
    else
    {
        yCWarning(ARV) << "Focus property not available";
    }

    //-- Start capturing images
    //-------------------------------------------------------------------------------

    //-- Initialization of buffer(s)
    if (stream)
    {
        g_object_unref(stream);
        stream = nullptr;
    }

#if ARAVIS_CHECK_VERSION(0, 7, 3)
    stream = arv_camera_create_stream(camera, nullptr, nullptr, nullptr);
#else
    stream = arv_camera_create_stream(camera, nullptr, nullptr);
#endif

    if (stream == nullptr)
    {
        yCError(ARV) << "Could not create Aravis stream";
        return false;
    }

    g_object_set(stream, "socket-buffer", ARV_GV_STREAM_SOCKET_BUFFER_AUTO, "socket-buffer-size", 0, nullptr);
    g_object_set(stream, "packet-resend", ARV_GV_STREAM_PACKET_RESEND_NEVER, nullptr);
    g_object_set(stream, "packet-timeout", (unsigned) 40000, "frame-retention", (unsigned) 200000, nullptr);

#if ARAVIS_CHECK_VERSION(0, 7, 3)
    payload = arv_camera_get_payload(camera, nullptr);
#else
    payload = arv_camera_get_payload(camera);
#endif

    for (int i = 0; i < num_buffers; i++)
    {
        arv_stream_push_buffer(stream, arv_buffer_new(payload, nullptr));
    }

    //-- Start continuous acquisition
#if ARAVIS_CHECK_VERSION(0, 7, 3)
    arv_camera_set_acquisition_mode(camera, ARV_ACQUISITION_MODE_CONTINUOUS, nullptr);
    arv_device_set_string_feature_value(arv_camera_get_device(camera), "TriggerMode" , "Off", nullptr);
    arv_camera_start_acquisition(camera, nullptr);
#else
    arv_camera_set_acquisition_mode(camera, ARV_ACQUISITION_MODE_CONTINUOUS);
    arv_device_set_string_feature_value(arv_camera_get_device(camera), "TriggerMode" , "Off");
    arv_camera_start_acquisition(camera);
#endif

    yCInfo(ARV) << "Aravis Camera acquisition started!";
    return true;
}

bool AravisGigE::close()
{
    if (camera == nullptr)
    {
        yCError(ARV) << "Camera was not started!";
        return false;
    }

#if ARAVIS_CHECK_VERSION(0, 7, 3)
    arv_camera_stop_acquisition(camera, nullptr);
#else
    arv_camera_stop_acquisition(camera);
#endif
    yCInfo(ARV) << "Aravis Camera acquisition stopped!";

    //-- Cleanup
    if (stream)
    {
        g_object_unref(stream);
        stream = nullptr;
    }

    g_object_unref(camera);
    camera = nullptr;

    return true;
}
