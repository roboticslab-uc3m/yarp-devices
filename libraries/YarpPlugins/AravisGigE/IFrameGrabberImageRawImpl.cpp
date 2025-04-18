#include "AravisGigE.hpp"

#include <yarp/os/LogStream.h>

#include "LogComponent.hpp"

bool AravisGigE::getImage(yarp::sig::ImageOf<yarp::sig::PixelMono> &image)
{
    //-- Right now it is implemented as polling (grab + retrieve image)
    //-- I think it could be also implemented with callbacks with ArvStreamCallback

    //-- Grab frame (get raw image)
    //--------------------------------------------------------------------------------
    framebuffer = nullptr;

    if (stream == nullptr)
    {
        yCError(ARV) << "Stream was not initialized";
        return false;
    }

    ArvBuffer *arvBuffer = nullptr;
    int max_tries = 10;
    int tries = 0;
    int success = false;
    while (!success && tries < max_tries)
    {
        arvBuffer = arv_stream_timeout_pop_buffer(stream, 200000);
        if (arvBuffer != nullptr && arv_buffer_get_status(arvBuffer) != ARV_BUFFER_STATUS_SUCCESS)
        {
            arv_stream_push_buffer(stream, arvBuffer);
        }
        else
            success = true;
    }

    if (arvBuffer != nullptr && success)
    {
        size_t buffer_size;
        framebuffer = (void *)arv_buffer_get_data(arvBuffer, &buffer_size);
        arv_buffer_get_image_region(arvBuffer, &xoffset, &yoffset, &_width, &_height);
        frameID = arv_buffer_get_frame_id(arvBuffer);
        arv_stream_push_buffer(stream, arvBuffer);
    }
    else
    {
        yCError(ARV) << "Timeout! Could not grab frame...";
        return false;
    }

    //-- Retrieve frame (convert and send as yarp image)
    //--------------------------------------------------------------------------------
    if (framebuffer!=nullptr)
    {
        //-- Create a yarp image container according with the current pixel format
        yarp::sig::Image raw_image;
        if (pixelFormat != ARV_PIXEL_FORMAT_MONO_8)
        {
            yCError(ARV) << "Unsupported pixel format";
        }

        //-- Write data
        image.zero();
        image.resize(_width, _height);
        mempcpy(image.getRawImage(), framebuffer, _width*_height*image.getPixelSize());
    }
    else
    {
        yCError(ARV) << "Framebuffer is empty";
        return false;
    }

    return true;
}

int AravisGigE::height() const
{
    return _height;
}

int AravisGigE::width() const
{
    return _width;
}
