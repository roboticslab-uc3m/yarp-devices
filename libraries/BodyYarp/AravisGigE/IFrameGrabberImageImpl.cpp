#include "AravisGigE.hpp"

bool roboticslab::AravisGigE::getImage(yarp::sig::ImageOf<yarp::sig::PixelRgb>& image)
{
    //-- Right now it is implemented as polling (grab + retrieve image)
    //-- I think it could be also implemented with callbacks with ArvStreamCallback
    CD_DEBUG("This is the default interface!\n");

    //-- Grab frame (get raw image)
    //--------------------------------------------------------------------------------
     if (!getRawBuffer(NULL))
     {
         return false;
     }

    //-- Retrieve frame (convert and send as yarp image)
    //--------------------------------------------------------------------------------
    if (framebuffer!=NULL)
    {
        //-- Create a yarp image container according with the current pixel format
        yarp::sig::Image raw_image;
        if (pixelFormat == ARV_PIXEL_FORMAT_MONO_8)
        {
            raw_image = yarp::sig::ImageOf<yarp::sig::PixelMono> ();
        }
        else if (pixelFormat == ARV_PIXEL_FORMAT_MONO_16)
        {
            raw_image = yarp::sig::ImageOf<yarp::sig::PixelMono16> ();
        }
        else
        {
            CD_ERROR("Unsupported pixel format\n");
        }

        //-- Write data
        raw_image.zero();
        raw_image.resize(_width, _height);
        mempcpy(raw_image.getRawImage(), framebuffer, _width*_height*raw_image.getPixelSize());
        image.copy(raw_image);
    }
    else
    {
        CD_ERROR("Framebuffer is empty\n");
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
