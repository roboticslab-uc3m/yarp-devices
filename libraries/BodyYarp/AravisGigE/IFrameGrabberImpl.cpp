#include "AravisGigE.hpp"

bool roboticslab::AravisGigE::getRawBuffer(unsigned char *buffer)
{
    //-- Right now it is implemented as polling
    //-- I think it could be also implemented with callbacks with ArvStreamCallback
    CD_DEBUG("This is the default interface!\n");

    //-- Grab frame (get raw image)
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

    buffer = (unsigned char*)framebuffer;
    return true;
}

int roboticslab::AravisGigE::getRawBufferSize()
{
    if (pixelFormat == ARV_PIXEL_FORMAT_MONO_8 || pixelFormat == ARV_PIXEL_FORMAT_MONO_16)
    {
        return _width*_height;
    }
    else
    {
        CD_ERROR("Unsupported pixel format\n");
        return -1;
    }
}
