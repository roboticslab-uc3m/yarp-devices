#include "AravisGigE.hpp"

#include <yarp/os/LogStream.h>

bool roboticslab::AravisGigE::getImage(yarp::sig::ImageOf<yarp::sig::PixelMono> &image)
{
    //-- Right now it is implemented as polling (grab + retrieve image)
    //-- I think it could be also implemented with callbacks with ArvStreamCallback

    //-- Grab frame (get raw image)
    //--------------------------------------------------------------------------------
    framebuffer = NULL;

    if (stream == NULL)
    {
        yError() << "Stream was not initialized";
        return false;
    }

    ArvBuffer *arvBuffer = NULL;
    int max_tries = 10;
    int tries = 0;
    int success = false;
    while (!success && tries < max_tries)
    {
        arvBuffer = arv_stream_timeout_pop_buffer(stream, 200000);
        if (arvBuffer != NULL && arv_buffer_get_status(arvBuffer) != ARV_BUFFER_STATUS_SUCCESS)
        {
            arv_stream_push_buffer(stream, arvBuffer);
        }
        else
            success = true;
    }

    if (arvBuffer != NULL && success)
    {
        size_t buffer_size;
        framebuffer = (void *)arv_buffer_get_data(arvBuffer, &buffer_size);
        arv_buffer_get_image_region(arvBuffer, &xoffset, &yoffset, &_width, &_height);
        frameID = arv_buffer_get_frame_id(arvBuffer);
        arv_stream_push_buffer(stream, arvBuffer);
    }
    else
    {
        yError() << "Timeout! Could not grab frame...";
        return false;
    }

    //-- Retrieve frame (convert and send as yarp image)
    //--------------------------------------------------------------------------------
    if (framebuffer!=NULL)
    {
        //-- Create a yarp image container according with the current pixel format
        yarp::sig::Image raw_image;
        if (pixelFormat != ARV_PIXEL_FORMAT_MONO_8)
        {
            yError() << "Unsupported pixel format";
        }

        //-- Write data
        image.zero();
        image.resize(_width, _height);
        mempcpy(image.getRawImage(), framebuffer, _width*_height*image.getPixelSize());
    }
    else
    {
        yError() << "Framebuffer is empty";
        return false;
    }

    return true;
}

int roboticslab::AravisGigE::height() const
{
    return _height;
}

int roboticslab::AravisGigE::width() const
{
    return _width;
}
