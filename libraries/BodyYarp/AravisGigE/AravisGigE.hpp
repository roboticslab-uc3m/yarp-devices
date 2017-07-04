#ifndef __ARAVIS_GIGE__
#define __ARAVIS_GIGE__

#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/FrameGrabberInterfaces.h>

#include <arv.h>

#include "ColorDebug.hpp"

namespace roboticslab {

 /**
 * @ingroup AravisGigE
 * @brief Implementation for GigE cameras using Aravis as driver.
 *
 */

class AravisGigE : public yarp::dev::DeviceDriver, public yarp::dev::IFrameGrabberImage,
        public yarp::dev::IFrameGrabberControls2
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
//        virtual bool setIris(int v);

        virtual int getZoom();
        virtual int getFocus();
//        virtual int getIris();

        // ---------- IFrameGrabberControls2 Declarations. Implementation in IFrameGrabberControls2Impl.cpp ---------
        virtual bool getCameraDescription(CameraDescriptor *camera);
        virtual bool hasFeature(int feature, bool *hasFeature);
        virtual bool setFeature(int feature, double value);
        virtual bool getFeature(int feature, double *value);
        virtual bool setFeature(int feature, double value1, double value2);
        virtual bool getFeature(int feature, double *value1, double *value2);
        virtual bool hasOnOff(int feature, bool *HasOnOff);
        virtual bool setActive(int feature, bool onoff);
        virtual bool getActive(int feature, bool *isActive);
        virtual bool hasAuto(int feature, bool *hasAuto);
        virtual bool hasManual(int feature, bool *hasManual);
        virtual bool hasOnePush(int feature, bool *hasOnePush);
        virtual bool setMode(int feature, FeatureMode mode);
        virtual bool getMode(int feature, FeatureMode *mode);
        virtual bool setOnePush(int feature);

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
        bool zoomAvailable;
        gint64 zoomMin;                            // Camera zoom minimum value
        gint64 zoomMax;                            // Camera zoom maximum value
        bool focusAvailable;
        gint64 focusMin;                           // Camera focus minimum value
        gint64 focusMax;                           // Camera focus maximum value
        bool irisAvailable;
        gint64 irisMin;                            // Camera iris minimum value
        gint64 irisMax;                            // Camera iris maximum value
};


}
#endif // __ARAVIS_GIGE__
