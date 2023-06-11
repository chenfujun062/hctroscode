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

#ifndef LV_ATTRIBUTE_IMG_IDB_HINT_MUTE
#define LV_ATTRIBUTE_IMG_IDB_HINT_MUTE
#endif

const LV_ATTRIBUTE_MEM_ALIGN LV_ATTRIBUTE_LARGE_CONST LV_ATTRIBUTE_IMG_IDB_HINT_MUTE uint8_t IDB_Hint_mute_map[] = {
#if LV_COLOR_DEPTH == 1 || LV_COLOR_DEPTH == 8
  /*Pixel format: Red: 3 bit, Green: 3 bit, Blue: 2 bit*/
  0xe3, 0xe3, 0xe3, 0xe3, 0xe3, 0xe3, 0xe3, 0xe3, 0xe3, 0xe3, 0xe3, 0xe3, 0xe3, 0xe3, 
  0xe3, 0xe3, 0xe3, 0xe3, 0xe3, 0xe3, 0x92, 0x6e, 0xe3, 0xe3, 0xe3, 0xe3, 0xe3, 0xe3, 
  0xe3, 0xe3, 0xe3, 0xe3, 0xe3, 0xb6, 0xff, 0x92, 0x6e, 0x6d, 0xe3, 0x49, 0x92, 0xe3, 
  0xe3, 0xe3, 0xe3, 0xe3, 0xdb, 0xff, 0xff, 0x92, 0xe3, 0xdb, 0xe3, 0xb6, 0x92, 0xe3, 
  0xe3, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x92, 0xe3, 0xb7, 0xb6, 0xdb, 0xe3, 0xe3, 
  0xe3, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x92, 0xe3, 0x6e, 0xff, 0xb7, 0xe3, 0xe3, 
  0xe3, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x92, 0xe3, 0xe3, 0xff, 0x92, 0xe3, 0xe3, 
  0xe3, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x92, 0xe3, 0x6e, 0xff, 0xdb, 0xe3, 0xe3, 
  0xe3, 0xb7, 0xb7, 0xdb, 0xff, 0xff, 0xff, 0x92, 0xe3, 0xdb, 0x92, 0xdb, 0xe3, 0xe3, 
  0xe3, 0xe3, 0xe3, 0xe3, 0x92, 0xff, 0xff, 0x92, 0xe3, 0xdb, 0xe3, 0x92, 0xb7, 0xe3, 
  0xe3, 0xe3, 0xe3, 0xe3, 0xe3, 0x49, 0xdb, 0x92, 0xe3, 0x6d, 0xe3, 0xe3, 0x92, 0xe3, 
  0xe3, 0xe3, 0xe3, 0xe3, 0xe3, 0xe3, 0xe3, 0x6d, 0xe3, 0xe3, 0xe3, 0xe3, 0xe3, 0xe3, 
  0xe3, 0xe3, 0xe3, 0xe3, 0xe3, 0xe3, 0xe3, 0xe3, 0xe3, 0xe3, 0xe3, 0xe3, 0xe3, 0xe3, 
#endif
#if LV_COLOR_DEPTH == 16 && LV_COLOR_16_SWAP == 0
  /*Pixel format: Red: 5 bit, Green: 6 bit, Blue: 5 bit*/
  0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 
  0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0xcf, 0x7b, 0x8e, 0x73, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 
  0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0xf3, 0x9c, 0x9e, 0xf7, 0x10, 0x84, 0x4d, 0x6b, 0x0c, 0x63, 0x1f, 0xf8, 0x08, 0x42, 0x72, 0x94, 0x1f, 0xf8, 
  0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0xd7, 0xbd, 0x7d, 0xef, 0x9e, 0xf7, 0x30, 0x84, 0x1f, 0xf8, 0x79, 0xce, 0x1f, 0xf8, 0xd3, 0x9c, 0x51, 0x8c, 0x1f, 0xf8, 
  0x1f, 0xf8, 0x7e, 0xf7, 0x7e, 0xf7, 0x7e, 0xf7, 0x9e, 0xf7, 0x9e, 0xf7, 0x9e, 0xf7, 0x30, 0x84, 0x1f, 0xf8, 0x76, 0xb5, 0xf3, 0x9c, 0x7a, 0xd6, 0x1f, 0xf8, 0x1f, 0xf8, 
  0x1f, 0xf8, 0x9e, 0xf7, 0x9e, 0xf7, 0x9e, 0xf7, 0x9e, 0xf7, 0x9e, 0xf7, 0x9e, 0xf7, 0x30, 0x84, 0x1f, 0xf8, 0x0c, 0x63, 0x5d, 0xef, 0x35, 0xad, 0x1f, 0xf8, 0x1f, 0xf8, 
  0x1f, 0xf8, 0x9e, 0xf7, 0x9e, 0xf7, 0x9e, 0xf7, 0x9e, 0xf7, 0x9e, 0xf7, 0x9e, 0xf7, 0x30, 0x84, 0x1f, 0xf8, 0x1f, 0xf8, 0x3d, 0xef, 0xaf, 0x7b, 0x1f, 0xf8, 0x1f, 0xf8, 
  0x1f, 0xf8, 0x9e, 0xf7, 0x9e, 0xf7, 0x9e, 0xf7, 0x9e, 0xf7, 0x9e, 0xf7, 0x9e, 0xf7, 0x30, 0x84, 0x1f, 0xf8, 0x4d, 0x6b, 0x5d, 0xef, 0xf7, 0xbd, 0x1f, 0xf8, 0x1f, 0xf8, 
  0x1f, 0xf8, 0x76, 0xb5, 0x76, 0xb5, 0xb7, 0xbd, 0x5d, 0xef, 0x9e, 0xf7, 0x9e, 0xf7, 0x30, 0x84, 0x1f, 0xf8, 0xf8, 0xc5, 0xaf, 0x7b, 0x18, 0xc6, 0x1f, 0xf8, 0x1f, 0xf8, 
  0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x10, 0x84, 0x3c, 0xe7, 0x9e, 0xf7, 0x30, 0x84, 0x1f, 0xf8, 0x9a, 0xd6, 0x1f, 0xf8, 0xef, 0x7b, 0x34, 0xa5, 0x1f, 0xf8, 
  0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x49, 0x4a, 0x9a, 0xd6, 0x10, 0x84, 0x1f, 0xf8, 0xab, 0x5a, 0x1f, 0xf8, 0x1f, 0xf8, 0xcf, 0x7b, 0x1f, 0xf8, 
  0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0xaa, 0x52, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 
  0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 
#endif
#if LV_COLOR_DEPTH == 16 && LV_COLOR_16_SWAP != 0
  /*Pixel format: Red: 5 bit, Green: 6 bit, Blue: 5 bit BUT the 2 bytes are swapped*/
  0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 
  0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0x7b, 0xcf, 0x73, 0x8e, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 
  0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0x9c, 0xf3, 0xf7, 0x9e, 0x84, 0x10, 0x6b, 0x4d, 0x63, 0x0c, 0xf8, 0x1f, 0x42, 0x08, 0x94, 0x72, 0xf8, 0x1f, 
  0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xbd, 0xd7, 0xef, 0x7d, 0xf7, 0x9e, 0x84, 0x30, 0xf8, 0x1f, 0xce, 0x79, 0xf8, 0x1f, 0x9c, 0xd3, 0x8c, 0x51, 0xf8, 0x1f, 
  0xf8, 0x1f, 0xf7, 0x7e, 0xf7, 0x7e, 0xf7, 0x7e, 0xf7, 0x9e, 0xf7, 0x9e, 0xf7, 0x9e, 0x84, 0x30, 0xf8, 0x1f, 0xb5, 0x76, 0x9c, 0xf3, 0xd6, 0x7a, 0xf8, 0x1f, 0xf8, 0x1f, 
  0xf8, 0x1f, 0xf7, 0x9e, 0xf7, 0x9e, 0xf7, 0x9e, 0xf7, 0x9e, 0xf7, 0x9e, 0xf7, 0x9e, 0x84, 0x30, 0xf8, 0x1f, 0x63, 0x0c, 0xef, 0x5d, 0xad, 0x35, 0xf8, 0x1f, 0xf8, 0x1f, 
  0xf8, 0x1f, 0xf7, 0x9e, 0xf7, 0x9e, 0xf7, 0x9e, 0xf7, 0x9e, 0xf7, 0x9e, 0xf7, 0x9e, 0x84, 0x30, 0xf8, 0x1f, 0xf8, 0x1f, 0xef, 0x3d, 0x7b, 0xaf, 0xf8, 0x1f, 0xf8, 0x1f, 
  0xf8, 0x1f, 0xf7, 0x9e, 0xf7, 0x9e, 0xf7, 0x9e, 0xf7, 0x9e, 0xf7, 0x9e, 0xf7, 0x9e, 0x84, 0x30, 0xf8, 0x1f, 0x6b, 0x4d, 0xef, 0x5d, 0xbd, 0xf7, 0xf8, 0x1f, 0xf8, 0x1f, 
  0xf8, 0x1f, 0xb5, 0x76, 0xb5, 0x76, 0xbd, 0xb7, 0xef, 0x5d, 0xf7, 0x9e, 0xf7, 0x9e, 0x84, 0x30, 0xf8, 0x1f, 0xc5, 0xf8, 0x7b, 0xaf, 0xc6, 0x18, 0xf8, 0x1f, 0xf8, 0x1f, 
  0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0x84, 0x10, 0xe7, 0x3c, 0xf7, 0x9e, 0x84, 0x30, 0xf8, 0x1f, 0xd6, 0x9a, 0xf8, 0x1f, 0x7b, 0xef, 0xa5, 0x34, 0xf8, 0x1f, 
  0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0x4a, 0x49, 0xd6, 0x9a, 0x84, 0x10, 0xf8, 0x1f, 0x5a, 0xab, 0xf8, 0x1f, 0xf8, 0x1f, 0x7b, 0xcf, 0xf8, 0x1f, 
  0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0x52, 0xaa, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 
  0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 
#endif
#if LV_COLOR_DEPTH == 32
  /*Pixel format: Fix 0xFF: 8 bit, Red: 8 bit, Green: 8 bit, Blue: 8 bit*/
  0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 
  0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0x77, 0x77, 0x77, 0xff, 0x6e, 0x6e, 0x6e, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 
  0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0x9b, 0x9b, 0x9b, 0xff, 0xf0, 0xf0, 0xf0, 0xff, 0x81, 0x81, 0x81, 0xff, 0x66, 0x66, 0x66, 0xff, 0x5f, 0x5f, 0x5f, 0xff, 0xff, 0x00, 0xff, 0xff, 0x3e, 0x3e, 0x3e, 0xff, 0x8d, 0x8d, 0x8d, 0xff, 0xff, 0x00, 0xff, 0xff, 
  0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0xb7, 0xb7, 0xb7, 0xff, 0xea, 0xea, 0xea, 0xff, 0xf0, 0xf0, 0xf0, 0xff, 0x82, 0x82, 0x82, 0xff, 0xff, 0x00, 0xff, 0xff, 0xcb, 0xcb, 0xcb, 0xff, 0xff, 0x00, 0xff, 0xff, 0x96, 0x96, 0x96, 0xff, 0x87, 0x87, 0x87, 0xff, 0xff, 0x00, 0xff, 0xff, 
  0xff, 0x00, 0xff, 0xff, 0xed, 0xed, 0xed, 0xff, 0xec, 0xec, 0xec, 0xff, 0xec, 0xec, 0xec, 0xff, 0xef, 0xef, 0xef, 0xff, 0xef, 0xef, 0xef, 0xff, 0xf0, 0xf0, 0xf0, 0xff, 0x82, 0x82, 0x82, 0xff, 0xff, 0x00, 0xff, 0xff, 0xad, 0xad, 0xad, 0xff, 0x9b, 0x9b, 0x9b, 0xff, 0xcd, 0xcd, 0xcd, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 
  0xff, 0x00, 0xff, 0xff, 0xef, 0xef, 0xef, 0xff, 0xef, 0xef, 0xef, 0xff, 0xef, 0xef, 0xef, 0xff, 0xef, 0xef, 0xef, 0xff, 0xef, 0xef, 0xef, 0xff, 0xf0, 0xf0, 0xf0, 0xff, 0x82, 0x82, 0x82, 0xff, 0xff, 0x00, 0xff, 0xff, 0x60, 0x60, 0x60, 0xff, 0xe7, 0xe7, 0xe7, 0xff, 0xa5, 0xa5, 0xa5, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 
  0xff, 0x00, 0xff, 0xff, 0xef, 0xef, 0xef, 0xff, 0xef, 0xef, 0xef, 0xff, 0xef, 0xef, 0xef, 0xff, 0xef, 0xef, 0xef, 0xff, 0xef, 0xef, 0xef, 0xff, 0xf0, 0xf0, 0xf0, 0xff, 0x82, 0x82, 0x82, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0xe5, 0xe5, 0xe5, 0xff, 0x75, 0x75, 0x75, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 
  0xff, 0x00, 0xff, 0xff, 0xef, 0xef, 0xef, 0xff, 0xef, 0xef, 0xef, 0xff, 0xef, 0xef, 0xef, 0xff, 0xef, 0xef, 0xef, 0xff, 0xef, 0xef, 0xef, 0xff, 0xf0, 0xf0, 0xf0, 0xff, 0x82, 0x82, 0x82, 0xff, 0xff, 0x00, 0xff, 0xff, 0x68, 0x68, 0x68, 0xff, 0xe7, 0xe7, 0xe7, 0xff, 0xba, 0xba, 0xba, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 
  0xff, 0x00, 0xff, 0xff, 0xad, 0xad, 0xad, 0xff, 0xad, 0xad, 0xad, 0xff, 0xb4, 0xb4, 0xb4, 0xff, 0xe9, 0xe9, 0xe9, 0xff, 0xef, 0xef, 0xef, 0xff, 0xf0, 0xf0, 0xf0, 0xff, 0x82, 0x82, 0x82, 0xff, 0xff, 0x00, 0xff, 0xff, 0xbc, 0xbc, 0xbc, 0xff, 0x75, 0x75, 0x75, 0xff, 0xc0, 0xc0, 0xc0, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 
  0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0x80, 0x80, 0x80, 0xff, 0xe3, 0xe3, 0xe3, 0xff, 0xf0, 0xf0, 0xf0, 0xff, 0x82, 0x82, 0x82, 0xff, 0xff, 0x00, 0xff, 0xff, 0xcf, 0xcf, 0xcf, 0xff, 0xff, 0x00, 0xff, 0xff, 0x7b, 0x7b, 0x7b, 0xff, 0xa2, 0xa2, 0xa2, 0xff, 0xff, 0x00, 0xff, 0xff, 
  0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0x47, 0x47, 0x47, 0xff, 0xce, 0xce, 0xce, 0xff, 0x81, 0x81, 0x81, 0xff, 0xff, 0x00, 0xff, 0xff, 0x55, 0x55, 0x55, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0x79, 0x79, 0x79, 0xff, 0xff, 0x00, 0xff, 0xff, 
  0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0x52, 0x52, 0x52, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 
  0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 
#endif
};

const lv_img_dsc_t IDB_Hint_mute = {
  .header.cf = LV_IMG_CF_TRUE_COLOR_CHROMA_KEYED,
  .header.always_zero = 0,
  .header.reserved = 0,
  .header.w = 14,
  .header.h = 13,
  .data_size = 182 * LV_COLOR_SIZE / 8,
  .data = IDB_Hint_mute_map,
};