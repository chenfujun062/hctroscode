#ifdef __has_include
    #if __has_include("lvgl.h")
        #ifndef LV_LVGL_H_INCLUDE_SIMPLE
            #define LV_LVGL_H_INCLUDE_SIMPLE
        #endif
    #endif
#endif

#if defined(LV_LVGL_H_INCLUDE_SIMPLE)
    #include "lvgl.h"
#else
    #include "lvgl/lvgl.h"
#endif


#ifndef LV_ATTRIBUTE_MEM_ALIGN
#define LV_ATTRIBUTE_MEM_ALIGN
#endif

#ifndef LV_ATTRIBUTE_IMG_IDB_HINT_MUSIC_ON
#define LV_ATTRIBUTE_IMG_IDB_HINT_MUSIC_ON
#endif

const LV_ATTRIBUTE_MEM_ALIGN LV_ATTRIBUTE_LARGE_CONST LV_ATTRIBUTE_IMG_IDB_HINT_MUSIC_ON uint8_t IDB_Hint_Music_On_map[] = {
#if LV_COLOR_DEPTH == 1 || LV_COLOR_DEPTH == 8
  /*Pixel format: Red: 3 bit, Green: 3 bit, Blue: 2 bit*/
  0xe3, 0xe3, 0xe3, 0xe3, 0xe3, 0xe3, 0xe3, 0xe3, 0xe3, 0xe3, 0xe3, 0xe3, 0xe3, 0xe3, 
  0xe3, 0xe3, 0xe3, 0xe3, 0x6d, 0xdb, 0xdb, 0xdb, 0xdb, 0xdb, 0xdb, 0xdb, 0x92, 0xe3, 
  0xe3, 0xe3, 0xe3, 0xe3, 0x6e, 0xff, 0xb6, 0xb6, 0xb6, 0xb6, 0xb6, 0xdb, 0xb6, 0xe3, 
  0xe3, 0xe3, 0xe3, 0xe3, 0x6e, 0xdb, 0x6d, 0x6d, 0x6d, 0x6d, 0x6d, 0xb7, 0xb6, 0xe3, 
  0xe3, 0xe3, 0xe3, 0xe3, 0x6e, 0xb7, 0xe3, 0xe3, 0xe3, 0xe3, 0xe3, 0x92, 0xb6, 0xe3, 
  0xe3, 0xe3, 0xe3, 0xe3, 0x6e, 0xb7, 0xe3, 0xe3, 0xe3, 0xe3, 0xe3, 0x92, 0xb6, 0xe3, 
  0xe3, 0xe3, 0xe3, 0xe3, 0x6e, 0xb7, 0xe3, 0xe3, 0xe3, 0xe3, 0xe3, 0x92, 0xb6, 0xe3, 
  0xe3, 0xe3, 0xe3, 0xe3, 0x6e, 0xb7, 0xe3, 0xe3, 0xe3, 0xe3, 0xe3, 0x92, 0xb6, 0xe3, 
  0xe3, 0xe3, 0x92, 0x92, 0xb7, 0xb7, 0xe3, 0xe3, 0xe3, 0x92, 0x92, 0xdb, 0xb6, 0xe3, 
  0xe3, 0x92, 0xff, 0xff, 0xff, 0xb7, 0xe3, 0xe3, 0xdb, 0xff, 0xff, 0xff, 0xb6, 0xe3, 
  0xe3, 0x92, 0xff, 0xff, 0xff, 0xb6, 0xe3, 0xe3, 0xdb, 0xff, 0xff, 0xff, 0x6e, 0xe3, 
  0xe3, 0xe3, 0x92, 0xb6, 0x92, 0xe3, 0xe3, 0xe3, 0x49, 0x92, 0xb6, 0x92, 0xe3, 0xe3, 
  0xe3, 0xe3, 0xe3, 0xe3, 0xe3, 0xe3, 0xe3, 0xe3, 0xe3, 0xe3, 0xe3, 0xe3, 0xe3, 0xe3, 
#endif
#if LV_COLOR_DEPTH == 16 && LV_COLOR_16_SWAP == 0
  /*Pixel format: Red: 5 bit, Green: 6 bit, Blue: 5 bit*/
  0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 
  0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0xaa, 0x52, 0xf8, 0xc5, 0x18, 0xc6, 0x18, 0xc6, 0x18, 0xc6, 0x18, 0xc6, 0x18, 0xc6, 0x18, 0xc6, 0xcf, 0x7b, 0x1f, 0xf8, 
  0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x4d, 0x6b, 0xba, 0xd6, 0xf3, 0x9c, 0xf3, 0x9c, 0xf3, 0x9c, 0xf3, 0x9c, 0xf3, 0x9c, 0x59, 0xce, 0xb2, 0x94, 0x1f, 0xf8, 
  0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x4d, 0x6b, 0x18, 0xc6, 0xab, 0x5a, 0xab, 0x5a, 0xab, 0x5a, 0xab, 0x5a, 0xab, 0x5a, 0x55, 0xad, 0xb2, 0x94, 0x1f, 0xf8, 
  0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x4d, 0x6b, 0x75, 0xad, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x30, 0x84, 0xb2, 0x94, 0x1f, 0xf8, 
  0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x4d, 0x6b, 0x76, 0xb5, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x31, 0x8c, 0xb2, 0x94, 0x1f, 0xf8, 
  0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x4d, 0x6b, 0x76, 0xb5, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x31, 0x8c, 0xb2, 0x94, 0x1f, 0xf8, 
  0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x4d, 0x6b, 0x76, 0xb5, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x31, 0x8c, 0xb2, 0x94, 0x1f, 0xf8, 
  0x1f, 0xf8, 0x1f, 0xf8, 0x8e, 0x73, 0x72, 0x94, 0x75, 0xad, 0x96, 0xb5, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0xcf, 0x7b, 0x71, 0x8c, 0xf7, 0xbd, 0xb2, 0x94, 0x1f, 0xf8, 
  0x1f, 0xf8, 0xef, 0x7b, 0x7e, 0xf7, 0x9e, 0xf7, 0x9e, 0xf7, 0x96, 0xb5, 0x1f, 0xf8, 0x1f, 0xf8, 0x96, 0xb5, 0x7e, 0xf7, 0x9e, 0xf7, 0x9e, 0xf7, 0xb2, 0x94, 0x1f, 0xf8, 
  0x1f, 0xf8, 0x71, 0x8c, 0x9e, 0xf7, 0x9e, 0xf7, 0x7e, 0xf7, 0x92, 0x94, 0x1f, 0xf8, 0x1f, 0xf8, 0x38, 0xc6, 0x9e, 0xf7, 0x9e, 0xf7, 0x7e, 0xf7, 0x6e, 0x73, 0x1f, 0xf8, 
  0x1f, 0xf8, 0x1f, 0xf8, 0x71, 0x8c, 0x14, 0xa5, 0x10, 0x84, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x08, 0x42, 0x92, 0x94, 0xf4, 0xa4, 0xef, 0x7b, 0x1f, 0xf8, 0x1f, 0xf8, 
  0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 
#endif
#if LV_COLOR_DEPTH == 16 && LV_COLOR_16_SWAP != 0
  /*Pixel format: Red: 5 bit, Green: 6 bit, Blue: 5 bit BUT the 2 bytes are swapped*/
  0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 
  0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0x52, 0xaa, 0xc5, 0xf8, 0xc6, 0x18, 0xc6, 0x18, 0xc6, 0x18, 0xc6, 0x18, 0xc6, 0x18, 0xc6, 0x18, 0x7b, 0xcf, 0xf8, 0x1f, 
  0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0x6b, 0x4d, 0xd6, 0xba, 0x9c, 0xf3, 0x9c, 0xf3, 0x9c, 0xf3, 0x9c, 0xf3, 0x9c, 0xf3, 0xce, 0x59, 0x94, 0xb2, 0xf8, 0x1f, 
  0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0x6b, 0x4d, 0xc6, 0x18, 0x5a, 0xab, 0x5a, 0xab, 0x5a, 0xab, 0x5a, 0xab, 0x5a, 0xab, 0xad, 0x55, 0x94, 0xb2, 0xf8, 0x1f, 
  0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0x6b, 0x4d, 0xad, 0x75, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0x84, 0x30, 0x94, 0xb2, 0xf8, 0x1f, 
  0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0x6b, 0x4d, 0xb5, 0x76, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0x8c, 0x31, 0x94, 0xb2, 0xf8, 0x1f, 
  0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0x6b, 0x4d, 0xb5, 0x76, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0x8c, 0x31, 0x94, 0xb2, 0xf8, 0x1f, 
  0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0x6b, 0x4d, 0xb5, 0x76, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0x8c, 0x31, 0x94, 0xb2, 0xf8, 0x1f, 
  0xf8, 0x1f, 0xf8, 0x1f, 0x73, 0x8e, 0x94, 0x72, 0xad, 0x75, 0xb5, 0x96, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0x7b, 0xcf, 0x8c, 0x71, 0xbd, 0xf7, 0x94, 0xb2, 0xf8, 0x1f, 
  0xf8, 0x1f, 0x7b, 0xef, 0xf7, 0x7e, 0xf7, 0x9e, 0xf7, 0x9e, 0xb5, 0x96, 0xf8, 0x1f, 0xf8, 0x1f, 0xb5, 0x96, 0xf7, 0x7e, 0xf7, 0x9e, 0xf7, 0x9e, 0x94, 0xb2, 0xf8, 0x1f, 
  0xf8, 0x1f, 0x8c, 0x71, 0xf7, 0x9e, 0xf7, 0x9e, 0xf7, 0x7e, 0x94, 0x92, 0xf8, 0x1f, 0xf8, 0x1f, 0xc6, 0x38, 0xf7, 0x9e, 0xf7, 0x9e, 0xf7, 0x7e, 0x73, 0x6e, 0xf8, 0x1f, 
  0xf8, 0x1f, 0xf8, 0x1f, 0x8c, 0x71, 0xa5, 0x14, 0x84, 0x10, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0x42, 0x08, 0x94, 0x92, 0xa4, 0xf4, 0x7b, 0xef, 0xf8, 0x1f, 0xf8, 0x1f, 
  0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 
#endif
#if LV_COLOR_DEPTH == 32
  /*Pixel format: Fix 0xFF: 8 bit, Red: 8 bit, Green: 8 bit, Blue: 8 bit*/
  0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 
  0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0x53, 0x53, 0x53, 0xff, 0xbd, 0xbd, 0xbd, 0xff, 0xbf, 0xbf, 0xbf, 0xff, 0xbf, 0xbf, 0xbf, 0xff, 0xbf, 0xbf, 0xbf, 0xff, 0xbf, 0xbf, 0xbf, 0xff, 0xbf, 0xbf, 0xbf, 0xff, 0xbf, 0xbf, 0xbf, 0xff, 0x77, 0x77, 0x77, 0xff, 0xff, 0x00, 0xff, 0xff, 
  0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0x66, 0x66, 0x66, 0xff, 0xd3, 0xd3, 0xd3, 0xff, 0x9a, 0x9a, 0x9a, 0xff, 0x9a, 0x9a, 0x9a, 0xff, 0x9a, 0x9a, 0x9a, 0xff, 0x9a, 0x9a, 0x9a, 0xff, 0x9a, 0x9a, 0x9a, 0xff, 0xc8, 0xc8, 0xc8, 0xff, 0x92, 0x92, 0x92, 0xff, 0xff, 0x00, 0xff, 0xff, 
  0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0x66, 0x66, 0x66, 0xff, 0xbf, 0xbf, 0xbf, 0xff, 0x55, 0x55, 0x55, 0xff, 0x55, 0x55, 0x55, 0xff, 0x55, 0x55, 0x55, 0xff, 0x55, 0x55, 0x55, 0xff, 0x55, 0x55, 0x55, 0xff, 0xa9, 0xa9, 0xa9, 0xff, 0x92, 0x92, 0x92, 0xff, 0xff, 0x00, 0xff, 0xff, 
  0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0x66, 0x66, 0x66, 0xff, 0xab, 0xab, 0xab, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0x83, 0x83, 0x83, 0xff, 0x92, 0x92, 0x92, 0xff, 0xff, 0x00, 0xff, 0xff, 
  0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0x66, 0x66, 0x66, 0xff, 0xad, 0xad, 0xad, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0x84, 0x84, 0x84, 0xff, 0x92, 0x92, 0x92, 0xff, 0xff, 0x00, 0xff, 0xff, 
  0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0x66, 0x66, 0x66, 0xff, 0xad, 0xad, 0xad, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0x84, 0x84, 0x84, 0xff, 0x92, 0x92, 0x92, 0xff, 0xff, 0x00, 0xff, 0xff, 
  0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0x66, 0x66, 0x66, 0xff, 0xad, 0xad, 0xad, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0x84, 0x84, 0x84, 0xff, 0x92, 0x92, 0x92, 0xff, 0xff, 0x00, 0xff, 0xff, 
  0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0x71, 0x71, 0x71, 0xff, 0x8c, 0x8c, 0x8c, 0xff, 0xaa, 0xaa, 0xaa, 0xff, 0xae, 0xae, 0xae, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0x76, 0x76, 0x76, 0xff, 0x8a, 0x8a, 0x8a, 0xff, 0xbb, 0xbb, 0xbb, 0xff, 0x92, 0x92, 0x92, 0xff, 0xff, 0x00, 0xff, 0xff, 
  0xff, 0x00, 0xff, 0xff, 0x7b, 0x7b, 0x7b, 0xff, 0xec, 0xec, 0xec, 0xff, 0xef, 0xef, 0xef, 0xff, 0xee, 0xee, 0xee, 0xff, 0xaf, 0xaf, 0xaf, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0xb1, 0xb1, 0xb1, 0xff, 0xec, 0xec, 0xec, 0xff, 0xef, 0xef, 0xef, 0xff, 0xee, 0xee, 0xee, 0xff, 0x93, 0x93, 0x93, 0xff, 0xff, 0x00, 0xff, 0xff, 
  0xff, 0x00, 0xff, 0xff, 0x8b, 0x8b, 0x8b, 0xff, 0xef, 0xef, 0xef, 0xff, 0xef, 0xef, 0xef, 0xff, 0xec, 0xec, 0xec, 0xff, 0x90, 0x90, 0x90, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0xc3, 0xc3, 0xc3, 0xff, 0xef, 0xef, 0xef, 0xff, 0xef, 0xef, 0xef, 0xff, 0xed, 0xed, 0xed, 0xff, 0x6d, 0x6d, 0x6d, 0xff, 0xff, 0x00, 0xff, 0xff, 
  0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0x8b, 0x8b, 0x8b, 0xff, 0x9f, 0x9f, 0x9f, 0xff, 0x7f, 0x7f, 0x7f, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0x41, 0x41, 0x41, 0xff, 0x8e, 0x8e, 0x8e, 0xff, 0x9d, 0x9d, 0x9d, 0xff, 0x7a, 0x7a, 0x7a, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 
  0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 
#endif
};

const lv_img_dsc_t IDB_Hint_Music_On = {
  .header.cf = LV_IMG_CF_TRUE_COLOR_CHROMA_KEYED,
  .header.always_zero = 0,
  .header.reserved = 0,
  .header.w = 14,
  .header.h = 13,
  .data_size = 182 * LV_COLOR_SIZE / 8,
  .data = IDB_Hint_Music_On_map,
};
