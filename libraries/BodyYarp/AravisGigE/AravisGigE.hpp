#ifndef __ARAVIS_GIGE__
#define __ARAVIS_GIGE__

#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/AudioVisualInterfaces.h>

#include <arv.h>

#include "ColorDebug.hpp"

namespace roboticslab {

 /**
 * @ingroup AravisGigE
 * @brief Implementation for GigE cameras using Aravis as driver.
 *
 */

class AravisGigE : public yarp::dev::DeviceDriver, public yarp::dev::IFrameGrabberImage
{
    public:

        AravisGigE() {}

        //  --------- DeviceDriver Declarations. Implementation in DeviceDriverImpl.cpp ---------
        virtual bool open(yarp::os::Searchable& config);
        virtual bool close();

        //  --------- IFrameGrabberImage Declarations. Implementation in IFrameGrabberImageImpl.cpp ---------
        virtual bool getImage(yarp::sig::ImageOf<yarp::sig::PixelRgb>& image);
        virtual int height() const;
        virtual int width() const;

        // ---------- ICameraLensControls Declarations. Implementation in ICameraLensControlsImpl.cpp ---------
        virtual bool setZoom(int v);
        virtual bool setFocus(int v);
        virtual bool setIris(int v);

        virtual int getZoom();
        virtual int getFocus();
        virtual int getIris();

        // ---------- IFrameGrabberControls Declarations. Implementation in IFrameGrabberControlsImpl.cpp ---------
        virtual bool setBrightness(double v);
        virtual bool setExposure(double v);
        virtual bool setSharpness(double v);
        virtual bool setWhiteBalance(double blue, double red);
        virtual bool setHue(double v);
        virtual bool setSaturation(double v);
        virtual bool setGamma(double v);
        virtual bool setShutter(double v);
        virtual bool setGain(double v);
        virtual bool setIris(double v);

        virtual double getBrightness();
        virtual double getExposure();
        virtual double getSharpness();
        virtual bool getWhiteBalance(double &blue, double &red);
        virtual double getHue();
        virtual double getSaturation();
        virtual double getGamma();
        virtual double getShutter();
        virtual double getGain();
        virtual double getIris();

    private:
        ArvCamera       *camera;                // Camera to control.
        ArvStream       *stream;                // Object for video stream reception.
        void            *framebuffer;           //

        unsigned int    payload;                // Width x height x Pixel width.

        int             widthMin;               // Camera sensor minium width.
        int             widthMax;               // Camera sensor maximum width.
        int             heightMin;              // Camera sensor minium height.
        int             heightMax;              // Camera sensor maximum height.
        bool            fpsAvailable;
        double          fpsMin;                 // Camera minium fps.
        double          fpsMax;                 // Camera maximum fps.
        bool            gainAvailable;
        double          gainMin;                // Camera minimum gain.
        double          gainMax;                // Camera maximum gain.
        bool            exposureAvailable;
        double          exposureMin;            // Camera's minimum exposure time.
        double          exposureMax;            // Camera's maximum exposure time.

        bool            controlExposure;        // Flag if automatic exposure shall be done by this SW
        bool            autoGain;
        double          targetGrey;             // Target grey value (mid grey))

        gint64          *pixelFormats;
        guint           pixelFormatsCnt;


        int             num_buffers;            // number of payload transmission buffers

        ArvPixelFormat  pixelFormat;            // pixel format

        int             xoffset;                // current frame region x offset
        int             yoffset;                // current frame region y offset
        int             _width;                  // current frame width of frame
        int             _height;                 // current frame height of image

        double          fps;                    // current value of fps
        double          exposure;               // current value of exposure time
        double          gain;                   // current value of gain
        double          midGrey;                // current value of mid grey (brightness)

        unsigned        frameID;                // current frame id
        unsigned        prevFrameID;

        //-- Lens Controls
        int zoomMin;                            // Camera zoom minimum value
        int zoomMax;                            // Camera zoom maximum value
        int focusMin;                           // Camera focus minimum value
        int focusMax;                           // Camera focus maximum value
        int irisMin;                            // Camera iris minimum value
        int irisMax;                            // Camera iris maximum value


};


}
#endif // __ARAVIS_GIGE__
