#include <linux/ioctl.h>
#include <linux/types.h>

#ifndef __JR3IOCTL__
#define __JR3IOCTL__

typedef struct force_array {
    int f[3];
    int m[3];
    int v[2];
} force_array;

typedef struct six_axis_array {
    int f[3];
    int m[3];
} six_axis_array;

#define JR3_IOC_MAGIC 'k'
#define JR3_TYPE JR3_IOC_MAGIC

#define IOCTL0_JR3_RESET              _IO ( JR3_TYPE, 0 )
#define IOCTL0_JR3_FILTER0            _IOR( JR3_TYPE, 1,  struct six_axis_array )
#define IOCTL0_JR3_FILTER1            _IOR( JR3_TYPE, 2,  struct six_axis_array )
#define IOCTL0_JR3_FILTER2            _IOR( JR3_TYPE, 3,  struct six_axis_array )
#define IOCTL0_JR3_FILTER3            _IOR( JR3_TYPE, 4,  struct six_axis_array )
#define IOCTL0_JR3_FILTER4            _IOR( JR3_TYPE, 5,  struct six_axis_array )
#define IOCTL0_JR3_FILTER5            _IOR( JR3_TYPE, 6,  struct six_axis_array )
#define IOCTL0_JR3_FILTER6            _IOR( JR3_TYPE, 7,  struct six_axis_array )
#define IOCTL0_JR3_ZEROOFFS           _IO ( JR3_TYPE, 8   )
#define IOCTL0_JR3_GET_FULL_SCALES    _IOR( JR3_TYPE, 9,  struct force_array )

#define IOCTL1_JR3_RESET              _IO ( JR3_TYPE, 10 )
#define IOCTL1_JR3_FILTER0            _IOR( JR3_TYPE, 11,  struct six_axis_array )
#define IOCTL1_JR3_FILTER1            _IOR( JR3_TYPE, 12,  struct six_axis_array )
#define IOCTL1_JR3_FILTER2            _IOR( JR3_TYPE, 13,  struct six_axis_array )
#define IOCTL1_JR3_FILTER3            _IOR( JR3_TYPE, 14,  struct six_axis_array )
#define IOCTL1_JR3_FILTER4            _IOR( JR3_TYPE, 15,  struct six_axis_array )
#define IOCTL1_JR3_FILTER5            _IOR( JR3_TYPE, 16,  struct six_axis_array )
#define IOCTL1_JR3_FILTER6            _IOR( JR3_TYPE, 17,  struct six_axis_array )
#define IOCTL1_JR3_ZEROOFFS           _IO ( JR3_TYPE, 18   )
#define IOCTL1_JR3_GET_FULL_SCALES    _IOR( JR3_TYPE, 19,  struct force_array )

#define IOCTL2_JR3_RESET              _IO ( JR3_TYPE, 20 )
#define IOCTL2_JR3_FILTER0            _IOR( JR3_TYPE, 21,  struct six_axis_array )
#define IOCTL2_JR3_FILTER1            _IOR( JR3_TYPE, 22,  struct six_axis_array )
#define IOCTL2_JR3_FILTER2            _IOR( JR3_TYPE, 23,  struct six_axis_array )
#define IOCTL2_JR3_FILTER3            _IOR( JR3_TYPE, 24,  struct six_axis_array )
#define IOCTL2_JR3_FILTER4            _IOR( JR3_TYPE, 25,  struct six_axis_array )
#define IOCTL2_JR3_FILTER5            _IOR( JR3_TYPE, 26,  struct six_axis_array )
#define IOCTL2_JR3_FILTER6            _IOR( JR3_TYPE, 27,  struct six_axis_array )
#define IOCTL2_JR3_ZEROOFFS           _IO ( JR3_TYPE, 28   )
#define IOCTL2_JR3_GET_FULL_SCALES    _IOR( JR3_TYPE, 29,  struct force_array )

#define IOCTL3_JR3_RESET              _IO ( JR3_TYPE, 30 )
#define IOCTL3_JR3_FILTER0            _IOR( JR3_TYPE, 31,  struct six_axis_array )
#define IOCTL3_JR3_FILTER1            _IOR( JR3_TYPE, 32,  struct six_axis_array )
#define IOCTL3_JR3_FILTER2            _IOR( JR3_TYPE, 33,  struct six_axis_array )
#define IOCTL3_JR3_FILTER3            _IOR( JR3_TYPE, 34,  struct six_axis_array )
#define IOCTL3_JR3_FILTER4            _IOR( JR3_TYPE, 35,  struct six_axis_array )
#define IOCTL3_JR3_FILTER5            _IOR( JR3_TYPE, 36,  struct six_axis_array )
#define IOCTL3_JR3_FILTER6            _IOR( JR3_TYPE, 37,  struct six_axis_array )
#define IOCTL3_JR3_ZEROOFFS           _IO ( JR3_TYPE, 38   )
#define IOCTL3_JR3_GET_FULL_SCALES    _IOR( JR3_TYPE, 39,  struct force_array )

#define IOCTL_JR3_MAXNR 40
#endif

