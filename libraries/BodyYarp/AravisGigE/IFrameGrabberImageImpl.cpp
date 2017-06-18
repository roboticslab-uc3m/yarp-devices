#include "AravisGigE.hpp"

bool roboticslab::AravisGigE::getImage(yarp::sig::ImageOf<yarp::sig::PixelRgb>& image)
{
    //-- Right now it is implemented as polling (grab + retrieve image)
    //-- I think it could be also implemented with callbacks with ArvStreamCallback

    //-- Grab frame
    //--------------------------------------------------------------------------------
    framebuffer = NULL;

    if (stream == NULL)
    {
        CD_ERROR("Stream was not initialized\n");
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
        CD_ERROR("Timeout! Could not grab frame...\n");
        return false;
    }

    //-- Retrieve frame and send as yarp image
    //--------------------------------------------------------------------------------

    //-- WIP

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
