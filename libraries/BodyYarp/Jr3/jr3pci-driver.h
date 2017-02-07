#define PCI_VENDOR_ID_JR3      0x1762 /* PCI Vendor ID */
///#define PCI_DEVICE_ID_JR3      0x1111 /* PCI Device ID - Single channel PCI. 6DOF sensor*/
#define PCI_DEVICE_ID_JR3      0x3114 /* PCI Device ID - Double channel PCI. Two 6DOF sensors or one 12DOF sensor*/

#define JR3_RAWCHANNELS   0x6000
#define JR3_COPYRIGHT     0x6040  /* copyrights                               */
#define JR3_SHUNTS        0x6060
#define JR3_DEFAULT_FS    0x6068
#define JR3_LOAD_ENVELOPE_NUM 0x606F
#define JR3_MIN_FULLSCALE 0x6070
#define JR3_TRANSFORM_NUM 0x6077
#define JR3_MAX_FULLSCALE 0x6078
#define JR3_PEAKADD       0x607F
#define JR3_FULLSCALE     0x6080  /* start of full scale data                  */
#define JR3_OFFSETS       0x6088
#define JR3_OFFSET_NUM    0x608E
#define JR3_VECTORAXES    0x608F
#define JR3_FILTER0       0x6090      /* start of filter 0 (unfiltered) data  */
#define JR3_FILTER1       0x6098      /* start of filter 1  */
#define JR3_FILTER2       0x60A0      /* start of filter 2  */
#define JR3_FILTER3       0x60A8      /* start of filter 3 (31 Hz) data       */
#define JR3_FILTER4       0x60B0      /* start of filter 5 (2 Hz) data        */
#define JR3_FILTER5       0x60B8      /* start of filter 5 (2 Hz) data        */
#define JR3_FILTER6       0x60C0      /* start of filter 5 (2 Hz) data        */
#define JR3_RATEDATA      0x60C8
#define JR3_COMMAND2      0x60E5      /* command word 2 location              */
#define JR3_COMMAND1      0x60E6      /* command word 1 location              */
#define JR3_COMMAND0      0x60E7      /* command word 0 location              */
#define JR3_COUNT_X       0x60EF  /* Count_x                                  */
#define JR3_CALIBINFO     0x60F4

// Commands Code
#define JR3_CMD_RESETOFFSETS 0x0800
#define JR3_CMD_SETFULLSCALES 0x0A00

#define JR3_DATA_OFFSET         0x6000   // in Board-address
#define JR3_RESET_ADDRESS       0x18000  // in Board-address
#define JR3_P8BIT_OFFSET        0x40000  // in Pci-offset
#define JR3_PD_DEVIDER          0x4000   // in Board-address
#define CARD_OFFSET		0x20000

