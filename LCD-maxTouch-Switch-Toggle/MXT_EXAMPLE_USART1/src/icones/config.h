
/*******************************************************************************
* image
* filename: unsaved
* name: config
*
* preset name: Color R8G8B8
* data block size: 8 bit(s), uint8_t
* RLE compression enabled: no
* conversion type: Color, not_used not_used
* bits per pixel: 24
*
* preprocess:
*  main scan direction: top_to_bottom
*  line scan direction: forward
*  inverse: no
*******************************************************************************/

/*
 typedef struct {
     const uint8_t *data;
     uint16_t width;
     uint16_t height;
     uint8_t dataSize;
     } tImage;
*/
#include <stdint.h>



static const uint8_t image_data_config[3072] = {
    0xff, 0xff, 0xff, 0xfb, 0xfb, 0xfb, 0xff, 0xff, 0xff, 0xfe, 0xfe, 0xfe, 0xff, 0xff, 0xff, 0xfc, 0xfc, 0xfc, 0xff, 0xff, 0xff, 0xfe, 0xfe, 0xfe, 0xff, 0xff, 0xff, 0xfe, 0xfe, 0xfe, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xb6, 0xb6, 0xb6, 0x18, 0x18, 0x18, 0x09, 0x09, 0x09, 0x07, 0x07, 0x07, 0x12, 0x12, 0x12, 0xa2, 0xa2, 0xa2, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfd, 0xfd, 0xfd, 0xfd, 0xfd, 0xfd, 0xff, 0xff, 0xff, 0xfe, 0xfe, 0xfe, 0xfd, 0xfd, 0xfd, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc, 0xfc, 0xfc, 0xff, 0xff, 0xff, 
    0xfd, 0xfd, 0xfd, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc, 0xfc, 0xfc, 0xff, 0xff, 0xff, 0xfe, 0xfe, 0xfe, 0xff, 0xff, 0xff, 0xfd, 0xfd, 0xfd, 0xff, 0xff, 0xff, 0xf9, 0xf9, 0xf9, 0xfe, 0xfe, 0xfe, 0xff, 0xff, 0xff, 0x6b, 0x6b, 0x6b, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x04, 0x04, 0x00, 0x00, 0x00, 0x51, 0x51, 0x51, 0xff, 0xff, 0xff, 0xfc, 0xfc, 0xfc, 0xfc, 0xfc, 0xfc, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe, 0xfe, 0xfe, 0xff, 0xff, 0xff, 0xfd, 0xfd, 0xfd, 0xfe, 0xfe, 0xfe, 0xfe, 0xfe, 0xfe, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe, 0xfe, 0xfe, 
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc, 0xfc, 0xfc, 0xfc, 0xfc, 0xfc, 0xff, 0xff, 0xff, 0xfe, 0xfe, 0xfe, 0xff, 0xff, 0xff, 0xfd, 0xfd, 0xfd, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfd, 0xfd, 0xfd, 0x27, 0x27, 0x27, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x17, 0x17, 0x17, 0xf5, 0xf5, 0xf5, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfb, 0xfb, 0xfb, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe, 0xfe, 0xfe, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe, 0xfe, 0xfe, 0xff, 0xff, 0xff, 
    0xfe, 0xfe, 0xfe, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xde, 0xde, 0xde, 0x53, 0x53, 0x53, 0xad, 0xad, 0xad, 0xff, 0xff, 0xff, 0xfc, 0xfc, 0xfc, 0xfd, 0xfd, 0xfd, 0xfb, 0xfb, 0xfb, 0x81, 0x81, 0x81, 0x02, 0x02, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x03, 0x03, 0x06, 0x06, 0x06, 0x7d, 0x7d, 0x7d, 0xff, 0xff, 0xff, 0xfe, 0xfe, 0xfe, 0xfc, 0xfc, 0xfc, 0xff, 0xff, 0xff, 0xd1, 0xd1, 0xd1, 0x6a, 0x6a, 0x6a, 0xdc, 0xdc, 0xdc, 0xff, 0xff, 0xff, 0xfb, 0xfb, 0xfb, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfd, 0xfd, 0xfd, 
    0xff, 0xff, 0xff, 0xfd, 0xfd, 0xfd, 0xfd, 0xfd, 0xfd, 0xff, 0xff, 0xff, 0xcd, 0xcd, 0xcd, 0x15, 0x15, 0x15, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x4b, 0x4b, 0x4b, 0x9d, 0x9d, 0x9d, 0x94, 0x94, 0x94, 0x49, 0x49, 0x49, 0x03, 0x03, 0x03, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x03, 0x03, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x04, 0x04, 0x4c, 0x4c, 0x4c, 0xa8, 0xa8, 0xa8, 0xb9, 0xb9, 0xb9, 0x70, 0x70, 0x70, 0x09, 0x09, 0x09, 0x01, 0x01, 0x01, 0x19, 0x19, 0x19, 0xcf, 0xcf, 0xcf, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfb, 0xfb, 0xfb, 0xff, 0xff, 0xff, 
    0xff, 0xff, 0xff, 0xfe, 0xfe, 0xfe, 0xff, 0xff, 0xff, 0xdd, 0xdd, 0xdd, 0x17, 0x17, 0x17, 0x00, 0x00, 0x00, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x05, 0x05, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x02, 0x02, 0x03, 0x03, 0x03, 0x00, 0x00, 0x00, 0x02, 0x02, 0x02, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x13, 0x13, 0x13, 0xdb, 0xdb, 0xdb, 0xfd, 0xfd, 0xfd, 0xff, 0xff, 0xff, 0xfc, 0xfc, 0xfc, 
    0xfe, 0xfe, 0xfe, 0xff, 0xff, 0xff, 0xfc, 0xfc, 0xfc, 0x69, 0x69, 0x69, 0x05, 0x05, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x02, 0x02, 0x06, 0x06, 0x06, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x02, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x03, 0x03, 0x02, 0x02, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x02, 0x02, 0x58, 0x58, 0x58, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfd, 0xfd, 0xfd, 
    0xfe, 0xfe, 0xfe, 0xfe, 0xfe, 0xfe, 0xff, 0xff, 0xff, 0xd2, 0xd2, 0xd2, 0x0a, 0x0a, 0x0a, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x03, 0x03, 0x03, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x03, 0x03, 0x00, 0x00, 0x00, 0x02, 0x02, 0x02, 0x00, 0x00, 0x00, 0x03, 0x03, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x03, 0x03, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x02, 0x02, 0x02, 0x00, 0x00, 0x00, 0x02, 0x02, 0x02, 0xba, 0xba, 0xba, 0xfd, 0xfd, 0xfd, 0xfe, 0xfe, 0xfe, 0xff, 0xff, 0xff, 
    0xfe, 0xfe, 0xfe, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x7a, 0x7a, 0x7a, 0x00, 0x00, 0x00, 0x02, 0x02, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x04, 0x04, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x02, 0x02, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x04, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x5f, 0x5f, 0x5f, 0xff, 0xff, 0xff, 0xfd, 0xfd, 0xfd, 0xfe, 0xfe, 0xfe, 0xff, 0xff, 0xff, 
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfd, 0xfd, 0xfd, 0xfc, 0xfc, 0xfc, 0xbb, 0xbb, 0xbb, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x05, 0x05, 0x05, 0x02, 0x02, 0x02, 0x03, 0x03, 0x03, 0x04, 0x04, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x02, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x05, 0x05, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x05, 0x05, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x04, 0x04, 0xa3, 0xa3, 0xa3, 0xfc, 0xfc, 0xfc, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
    0xfd, 0xfd, 0xfd, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfd, 0xfd, 0xfd, 0xaa, 0xaa, 0xaa, 0x05, 0x05, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x02, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x05, 0x05, 0x05, 0x00, 0x00, 0x00, 0x14, 0x14, 0x14, 0x42, 0x42, 0x42, 0x46, 0x46, 0x46, 0x11, 0x11, 0x11, 0x01, 0x01, 0x01, 0x02, 0x02, 0x02, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x02, 0x02, 0x02, 0x00, 0x00, 0x00, 0x05, 0x05, 0x05, 0x00, 0x00, 0x00, 0x05, 0x05, 0x05, 0x9b, 0x9b, 0x9b, 0xfd, 0xfd, 0xfd, 0xff, 0xff, 0xff, 0xfd, 0xfd, 0xfd, 0xff, 0xff, 0xff, 
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe, 0xfe, 0xfe, 0xff, 0xff, 0xff, 0x4f, 0x4f, 0x4f, 0x00, 0x00, 0x00, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x14, 0x14, 0x14, 0xa7, 0xa7, 0xa7, 0xfd, 0xfd, 0xfd, 0xfc, 0xfc, 0xfc, 0xfd, 0xfd, 0xfd, 0xfd, 0xfd, 0xfd, 0xab, 0xab, 0xab, 0x18, 0x18, 0x18, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x02, 0x02, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x49, 0x49, 0x49, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe, 0xfe, 0xfe, 0xfe, 0xfe, 0xfe, 
    0xfd, 0xfd, 0xfd, 0xfc, 0xfc, 0xfc, 0xf5, 0xf5, 0xf5, 0x73, 0x73, 0x73, 0x00, 0x00, 0x00, 0x02, 0x02, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x03, 0x03, 0x03, 0x17, 0x17, 0x17, 0xd9, 0xd9, 0xd9, 0xfd, 0xfd, 0xfd, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfd, 0xfd, 0xfd, 0xdf, 0xdf, 0xdf, 0x1d, 0x1d, 0x1d, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x04, 0x04, 0x00, 0x00, 0x00, 0x03, 0x03, 0x03, 0x04, 0x04, 0x04, 0x7a, 0x7a, 0x7a, 0xfb, 0xfb, 0xfb, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
    0xa3, 0xa3, 0xa3, 0x48, 0x48, 0x48, 0x15, 0x15, 0x15, 0x04, 0x04, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xa5, 0xa5, 0xa5, 0xfd, 0xfd, 0xfd, 0xff, 0xff, 0xff, 0xfd, 0xfd, 0xfd, 0xfd, 0xfd, 0xfd, 0xff, 0xff, 0xff, 0xfd, 0xfd, 0xfd, 0xff, 0xff, 0xff, 0xfd, 0xfd, 0xfd, 0xb0, 0xb0, 0xb0, 0x01, 0x01, 0x01, 0x02, 0x02, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x05, 0x05, 0x05, 0x27, 0x27, 0x27, 0x60, 0x60, 0x60, 0xb6, 0xb6, 0xb6, 
    0x17, 0x17, 0x17, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x02, 0x02, 0x00, 0x00, 0x00, 0x03, 0x03, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x02, 0x02, 0x15, 0x15, 0x15, 0xfc, 0xfc, 0xfc, 0xff, 0xff, 0xff, 0xfa, 0xfa, 0xfa, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe, 0xfe, 0xfe, 0xff, 0xff, 0xff, 0xfd, 0xfd, 0xfd, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x19, 0x19, 0x19, 0x02, 0x02, 0x02, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x03, 0x03, 0x03, 0x00, 0x00, 0x00, 0x02, 0x02, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x02, 0x02, 0x19, 0x19, 0x19, 
    0x05, 0x05, 0x05, 0x01, 0x01, 0x01, 0x06, 0x06, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x3f, 0x3f, 0x3f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc, 0xfc, 0xfc, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc, 0xfc, 0xfc, 0xff, 0xff, 0xff, 0xfd, 0xfd, 0xfd, 0xff, 0xff, 0xff, 0x4f, 0x4f, 0x4f, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x07, 0x07, 0x07, 
    0x06, 0x06, 0x06, 0x00, 0x00, 0x00, 0x02, 0x02, 0x02, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x03, 0x03, 0x00, 0x00, 0x00, 0x44, 0x44, 0x44, 0xff, 0xff, 0xff, 0xfe, 0xfe, 0xfe, 0xff, 0xff, 0xff, 0xfd, 0xfd, 0xfd, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe, 0xfe, 0xfe, 0xff, 0xff, 0xff, 0xfe, 0xfe, 0xfe, 0xff, 0xff, 0xff, 0x52, 0x52, 0x52, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x04, 0x04, 0x02, 0x02, 0x02, 0x08, 0x08, 0x08, 
    0x17, 0x17, 0x17, 0x02, 0x02, 0x02, 0x00, 0x00, 0x00, 0x02, 0x02, 0x02, 0x04, 0x04, 0x04, 0x00, 0x00, 0x00, 0x03, 0x03, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0x07, 0x07, 0x17, 0x17, 0x17, 0xf8, 0xf8, 0xf8, 0xff, 0xff, 0xff, 0xfc, 0xfc, 0xfc, 0xff, 0xff, 0xff, 0xfc, 0xfc, 0xfc, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc, 0xfc, 0xfc, 0xfd, 0xfd, 0xfd, 0x20, 0x20, 0x20, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x05, 0x05, 0x05, 0x03, 0x03, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x16, 0x16, 0x16, 
    0xb9, 0xb9, 0xb9, 0x63, 0x63, 0x63, 0x27, 0x27, 0x27, 0x03, 0x03, 0x03, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x02, 0x02, 0x02, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0xaf, 0xaf, 0xaf, 0xfb, 0xfb, 0xfb, 0xfc, 0xfc, 0xfc, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfd, 0xfd, 0xfd, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc, 0xfc, 0xfc, 0xbd, 0xbd, 0xbd, 0x03, 0x03, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x00, 0x00, 0x00, 0x03, 0x03, 0x03, 0x1c, 0x1c, 0x1c, 0x4f, 0x4f, 0x4f, 0xa5, 0xa5, 0xa5, 
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc, 0xfc, 0xfc, 0x7c, 0x7c, 0x7c, 0x06, 0x06, 0x06, 0x04, 0x04, 0x04, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1e, 0x1e, 0x1e, 0xe3, 0xe3, 0xe3, 0xff, 0xff, 0xff, 0xfd, 0xfd, 0xfd, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe, 0xfe, 0xfe, 0xff, 0xff, 0xff, 0xea, 0xea, 0xea, 0x1e, 0x1e, 0x1e, 0x00, 0x00, 0x00, 0x03, 0x03, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x03, 0x03, 0x03, 0x7a, 0x7a, 0x7a, 0xf9, 0xf9, 0xf9, 0xfe, 0xfe, 0xfe, 0xff, 0xff, 0xff, 
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x46, 0x46, 0x46, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x02, 0x02, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x19, 0x19, 0x19, 0xb4, 0xb4, 0xb4, 0xff, 0xff, 0xff, 0xfe, 0xfe, 0xfe, 0xff, 0xff, 0xff, 0xfc, 0xfc, 0xfc, 0xb8, 0xb8, 0xb8, 0x23, 0x23, 0x23, 0x00, 0x00, 0x00, 0x05, 0x05, 0x05, 0x02, 0x02, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x03, 0x03, 0x00, 0x00, 0x00, 0x50, 0x50, 0x50, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe, 0xfe, 0xfe, 
    0xfd, 0xfd, 0xfd, 0xfc, 0xfc, 0xfc, 0xff, 0xff, 0xff, 0xfe, 0xfe, 0xfe, 0x9d, 0x9d, 0x9d, 0x05, 0x05, 0x05, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x02, 0x02, 0x02, 0x00, 0x00, 0x00, 0x02, 0x02, 0x02, 0x1e, 0x1e, 0x1e, 0x4f, 0x4f, 0x4f, 0x53, 0x53, 0x53, 0x20, 0x20, 0x20, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x02, 0x02, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x04, 0x04, 0x03, 0x03, 0x03, 0x00, 0x00, 0x00, 0x03, 0x03, 0x03, 0xad, 0xad, 0xad, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc, 0xfc, 0xfc, 0xff, 0xff, 0xff, 
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfa, 0xfa, 0xfa, 0xa1, 0xa1, 0xa1, 0x04, 0x04, 0x04, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x02, 0x02, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x02, 0x02, 0x02, 0x03, 0x03, 0x03, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x05, 0x05, 0x05, 0xbd, 0xbd, 0xbd, 0xfd, 0xfd, 0xfd, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
    0xff, 0xff, 0xff, 0xfe, 0xfe, 0xfe, 0xfd, 0xfd, 0xfd, 0xff, 0xff, 0xff, 0x5e, 0x5e, 0x5e, 0x00, 0x00, 0x00, 0x02, 0x02, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x04, 0x04, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x03, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x02, 0x02, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x03, 0x03, 0x00, 0x00, 0x00, 0x7a, 0x7a, 0x7a, 0xff, 0xff, 0xff, 0xfe, 0xfe, 0xfe, 0xff, 0xff, 0xff, 0xfe, 0xfe, 0xfe, 
    0xff, 0xff, 0xff, 0xfe, 0xfe, 0xfe, 0xfd, 0xfd, 0xfd, 0xba, 0xba, 0xba, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x02, 0x02, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x03, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x04, 0x04, 0x00, 0x00, 0x00, 0x02, 0x02, 0x02, 0x00, 0x00, 0x00, 0x04, 0x04, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x04, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x02, 0x02, 0x02, 0x00, 0x00, 0x00, 0x0a, 0x0a, 0x0a, 0xd1, 0xd1, 0xd1, 0xff, 0xff, 0xff, 0xfe, 0xfe, 0xfe, 0xff, 0xff, 0xff, 
    0xfd, 0xfd, 0xfd, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x58, 0x58, 0x58, 0x02, 0x02, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x02, 0x02, 0x02, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x03, 0x03, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x04, 0x04, 0x00, 0x00, 0x00, 0x02, 0x02, 0x02, 0x00, 0x00, 0x00, 0x04, 0x04, 0x04, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x03, 0x03, 0x6a, 0x6a, 0x6a, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfd, 0xfd, 0xfd, 
    0xfe, 0xfe, 0xfe, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xde, 0xde, 0xde, 0x16, 0x16, 0x16, 0x02, 0x02, 0x02, 0x00, 0x00, 0x00, 0x02, 0x02, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x02, 0x02, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x04, 0x04, 0x02, 0x02, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x02, 0x02, 0x00, 0x00, 0x00, 0x03, 0x03, 0x03, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x14, 0x14, 0x14, 0xda, 0xda, 0xda, 0xfe, 0xfe, 0xfe, 0xfc, 0xfc, 0xfc, 0xff, 0xff, 0xff, 
    0xff, 0xff, 0xff, 0xfa, 0xfa, 0xfa, 0xff, 0xff, 0xff, 0xfe, 0xfe, 0xfe, 0xce, 0xce, 0xce, 0x17, 0x17, 0x17, 0x00, 0x00, 0x00, 0x0a, 0x0a, 0x0a, 0x75, 0x75, 0x75, 0xba, 0xba, 0xba, 0xab, 0xab, 0xab, 0x4f, 0x4f, 0x4f, 0x03, 0x03, 0x03, 0x00, 0x00, 0x00, 0x03, 0x03, 0x03, 0x01, 0x01, 0x01, 0x04, 0x04, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x03, 0x03, 0x49, 0x49, 0x49, 0x9a, 0x9a, 0x9a, 0x9b, 0x9b, 0x9b, 0x4d, 0x4d, 0x4d, 0x00, 0x00, 0x00, 0x03, 0x03, 0x03, 0x18, 0x18, 0x18, 0xcf, 0xcf, 0xcf, 0xff, 0xff, 0xff, 0xfd, 0xfd, 0xfd, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc, 0xfc, 0xfc, 0xff, 0xff, 0xff, 0xdd, 0xdd, 0xdd, 0x6c, 0x6c, 0x6c, 0xd4, 0xd4, 0xd4, 0xfd, 0xfd, 0xfd, 0xfe, 0xfe, 0xfe, 0xfd, 0xfd, 0xfd, 0xfd, 0xfd, 0xfd, 0x7e, 0x7e, 0x7e, 0x05, 0x05, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x03, 0x03, 0x02, 0x02, 0x02, 0x83, 0x83, 0x83, 0xfd, 0xfd, 0xfd, 0xfd, 0xfd, 0xfd, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xad, 0xad, 0xad, 0x52, 0x52, 0x52, 0xdc, 0xdc, 0xdc, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfb, 0xfb, 0xfb, 
    0xfe, 0xfe, 0xfe, 0xfb, 0xfb, 0xfb, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe, 0xfe, 0xfe, 0xfd, 0xfd, 0xfd, 0xfc, 0xfc, 0xfc, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfd, 0xfd, 0xfd, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf8, 0xf8, 0xf8, 0x19, 0x19, 0x19, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x28, 0x28, 0x28, 0xfd, 0xfd, 0xfd, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe, 0xfe, 0xfe, 0xff, 0xff, 0xff, 0xfe, 0xfe, 0xfe, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe, 0xfe, 0xfe, 0xfe, 0xfe, 0xfe, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfd, 0xfd, 0xfd, 0xff, 0xff, 0xff, 0xfe, 0xfe, 0xfe, 0xff, 0xff, 0xff, 0xfe, 0xfe, 0xfe, 0xfb, 0xfb, 0xfb, 0xfd, 0xfd, 0xfd, 0xfc, 0xfc, 0xfc, 0x50, 0x50, 0x50, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x02, 0x02, 0x02, 0x00, 0x00, 0x00, 0x6f, 0x6f, 0x6f, 0xff, 0xff, 0xff, 0xfb, 0xfb, 0xfb, 0xfc, 0xfc, 0xfc, 0xff, 0xff, 0xff, 0xfe, 0xfe, 0xfe, 0xfd, 0xfd, 0xfd, 0xfe, 0xfe, 0xfe, 0xff, 0xff, 0xff, 0xfc, 0xfc, 0xfc, 0xff, 0xff, 0xff, 0xfe, 0xfe, 0xfe, 0xff, 0xff, 0xff, 0xfc, 0xfc, 0xfc, 
    0xff, 0xff, 0xff, 0xfd, 0xfd, 0xfd, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfd, 0xfd, 0xfd, 0xfe, 0xfe, 0xfe, 0xff, 0xff, 0xff, 0xfe, 0xfe, 0xfe, 0xfd, 0xfd, 0xfd, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe, 0xfe, 0xfe, 0xff, 0xff, 0xff, 0xa3, 0xa3, 0xa3, 0x11, 0x11, 0x11, 0x08, 0x08, 0x08, 0x07, 0x07, 0x07, 0x18, 0x18, 0x18, 0xb6, 0xb6, 0xb6, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe, 0xfe, 0xfe, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfb, 0xfb, 0xfb, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc, 0xfc, 0xfc, 0xff, 0xff, 0xff
};
const tImage config = { image_data_config, 32, 32,
    8 };

