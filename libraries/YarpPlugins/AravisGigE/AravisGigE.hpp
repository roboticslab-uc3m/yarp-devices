#ifndef __ARAVIS_GIGE__
#define __ARAVIS_GIGE__

#include <map>

#include <yarp/conf/version.h>
#include <yarp/dev/DeviceDriver.h>

#if YARP_VERSION_MINOR >= 5
# include <yarp/dev/IFrameGrabber.h>
# include <yarp/dev/IFrameGrabberControls.h>
# include <yarp/dev/IFrameGrabberImageRaw.h>
#else
# include <yarp/dev/FrameGrabberInterfaces.h>
#endif

#include <arv.h>

namespace roboticslab {

/**
 * @ingroup YarpPlugins
 * @defgroup AravisGigE
 * @brief Contains roboticslab::AravisGigE.
 */

 /**
  * @ingroup AravisGigE
  * @brief Implementation for GigE cameras using Aravis as driver.
  */
class AravisGigE : public yarp::dev::DeviceDriver,
                   public yarp::dev::IFrameGrabberImageRaw,
                   public yarp::dev::IFrameGrabber,
                   public yarp::dev::IFrameGrabberControls
{
    public:

        AravisGigE() {}
        ~AravisGigE() { close(); }

        //  --------- DeviceDriver Declarations. Implementation in DeviceDriverImpl.cpp ---------
        virtual bool open(yarp::os::Searchable& config);
        virtual bool close();


        //  --------- IFrameGrabber Declarations. Implementation in IFrameGrabberImpl.cpp ---------
        virtual bool getRawBuffer(unsigned char *buffer);
        virtual int getRawBufferSize();
        //virtual int height() const; //-- Implemented by IFrameGrabberImageImpl
        //virtual int width() const; //-- Implemented by IFrameGrabberImageImpl

        //  --------- IFrameGrabberImageRaw Declarations. Implementation in IFrameGrabberImageRawImpl.cpp ---------
        virtual bool getImage(yarp::sig::ImageOf<yarp::sig::PixelMono>& image);
        virtual int height() const;
        virtual int width() const;

        // ---------- IFrameGrabberControls Declarations. Implementation in IFrameGrabberControlsImpl.cpp ---------
        virtual bool getCameraDescription(CameraDescriptor *camera);
        virtual bool hasFeature(int feature, bool *hasFeature);
        virtual bool setFeature(int feature, double value);
        virtual bool getFeature(int feature, double *value);
        virtual bool getFeatureLimits(int feature, double *minValue, double *maxValue);
        virtual bool setFeature(int feature, double value1, double value2);
        virtual bool getFeature(int feature, double *value1, double *value2);
        virtual bool getFeatureLimits(int feature, double *minValue1, double *maxValue1, double *minValue2, double *maxValue2);
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

        //-- IFrameGrabberControls2
        std::map<cameraFeature_id_t, const char*> yarp_arv_int_feature_map; //-- Map yarp features with aravis feature ids
        std::map<cameraFeature_id_t, const char*> yarp_arv_float_feat_map;
};

}

#endif // __ARAVIS_GIGE__
