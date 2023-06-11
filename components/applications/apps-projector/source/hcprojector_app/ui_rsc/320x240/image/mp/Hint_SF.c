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

#ifndef LV_ATTRIBUTE_IMG_HINT_SF
#define LV_ATTRIBUTE_IMG_HINT_SF
#endif

const LV_ATTRIBUTE_MEM_ALIGN LV_ATTRIBUTE_LARGE_CONST LV_ATTRIBUTE_IMG_HINT_SF uint8_t Hint_SF_map[] = {
#if LV_COLOR_DEPTH == 1 || LV_COLOR_DEPTH == 8
  /*Pixel format: Red: 3 bit, Green: 3 bit, Blue: 2 bit*/
  0xe3, 0xe3, 0xe3, 0xe3, 0xe3, 0xe3, 0xe3, 0xe3, 0xe3, 0xe3, 0xe3, 0xe3, 0xe3, 0xe3, 
  0xe3, 0x92, 0x92, 0xe3, 0xe3, 0xe3, 0x6d, 0xe3, 0xe3, 0xe3, 0xe3, 0xe3, 0xe3, 0xe3, 
  0xe3, 0xdb, 0xdb, 0xe3, 0xe3, 0xb6, 0xff, 0xdb, 0xe3, 0xe3, 0xe3, 0xe3, 0xe3, 0xe3, 
  0xe3, 0xdb, 0xdb, 0xe3, 0xe3, 0xb6, 0xff, 0xff, 0xdb, 0x6e, 0xe3, 0xe3, 0xe3, 0xe3, 
  0xe3, 0xdb, 0xdb, 0xe3, 0xe3, 0xb6, 0xff, 0xff, 0xff, 0xff, 0xb7, 0xe3, 0xe3, 0xe3, 
  0xe3, 0xdb, 0xdb, 0xe3, 0xe3, 0xb6, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x6e, 0xe3, 
  0xe3, 0xdb, 0xdb, 0xe3, 0xe3, 0xb6, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xdb, 0xe3, 
  0xe3, 0xdb, 0xdb, 0xe3, 0xe3, 0xb6, 0xff, 0xff, 0xff, 0xff, 0xff, 0xdb, 0xe3, 0xe3, 
  0xe3, 0xdb, 0xdb, 0xe3, 0xe3, 0xb6, 0xff, 0xff, 0xff, 0xdb, 0x6d, 0xe3, 0xe3, 0xe3, 
  0xe3, 0xdb, 0xdb, 0xe3, 0xe3, 0xb6, 0xff, 0xff, 0xb7, 0xe3, 0xe3, 0xe3, 0xe3, 0xe3, 
  0xe3, 0xdb, 0xdb, 0xe3, 0xe3, 0x92, 0xdb, 0x6d, 0xe3, 0xe3, 0xe3, 0xe3, 0xe3, 0xe3, 
  0xe3, 0x92, 0x92, 0xe3, 0xe3, 0xe3, 0xe3, 0xe3, 0xe3, 0xe3, 0xe3, 0xe3, 0xe3, 0xe3, 
  0xe3, 0xe3, 0xe3, 0xe3, 0xe3, 0xe3, 0xe3, 0xe3, 0xe3, 0xe3, 0xe3, 0xe3, 0xe3, 0xe3, 
#endif
#if LV_COLOR_DEPTH == 16 && LV_COLOR_16_SWAP == 0
  /*Pixel format: Red: 5 bit, Green: 6 bit, Blue: 5 bit*/
  0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 
  0x1f, 0xf8, 0xaf, 0x7b, 0xaf, 0x7b, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0xab, 0x5a, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 
  0x1f, 0xf8, 0xd7, 0xbd, 0xd7, 0xbd, 0x1f, 0xf8, 0x1f, 0xf8, 0xb2, 0x94, 0x5d, 0xef, 0x96, 0xb5, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 
  0x1f, 0xf8, 0xd7, 0xbd, 0xd7, 0xbd, 0x1f, 0xf8, 0x1f, 0xf8, 0xb2, 0x94, 0x7e, 0xf7, 0x7e, 0xf7, 0x59, 0xce, 0x6d, 0x6b, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 
  0x1f, 0xf8, 0xd7, 0xbd, 0xd7, 0xbd, 0x1f, 0xf8, 0x1f, 0xf8, 0xb2, 0x94, 0x7e, 0xf7, 0x7e, 0xf7, 0x7e, 0xf7, 0x7e, 0xf7, 0x96, 0xb5, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 
  0x1f, 0xf8, 0xd7, 0xbd, 0xd7, 0xbd, 0x1f, 0xf8, 0x1f, 0xf8, 0xb2, 0x94, 0x7e, 0xf7, 0x7e, 0xf7, 0x7e, 0xf7, 0x7e, 0xf7, 0x7d, 0xef, 0xbb, 0xde, 0x2d, 0x6b, 0x1f, 0xf8, 
  0x1f, 0xf8, 0xd7, 0xbd, 0xd7, 0xbd, 0x1f, 0xf8, 0x1f, 0xf8, 0xb2, 0x94, 0x7e, 0xf7, 0x7e, 0xf7, 0x7e, 0xf7, 0x7e, 0xf7, 0x7e, 0xf7, 0x7e, 0xf7, 0x59, 0xce, 0x1f, 0xf8, 
  0x1f, 0xf8, 0xd7, 0xbd, 0xd7, 0xbd, 0x1f, 0xf8, 0x1f, 0xf8, 0xb2, 0x94, 0x7e, 0xf7, 0x7e, 0xf7, 0x7e, 0xf7, 0x7e, 0xf7, 0x7d, 0xef, 0xd7, 0xbd, 0x1f, 0xf8, 0x1f, 0xf8, 
  0x1f, 0xf8, 0xd7, 0xbd, 0xd7, 0xbd, 0x1f, 0xf8, 0x1f, 0xf8, 0xb2, 0x94, 0x7e, 0xf7, 0x7e, 0xf7, 0x7d, 0xef, 0x9a, 0xd6, 0xec, 0x62, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 
  0x1f, 0xf8, 0xd7, 0xbd, 0xd7, 0xbd, 0x1f, 0xf8, 0x1f, 0xf8, 0xb2, 0x94, 0x7e, 0xf7, 0x7e, 0xf7, 0x34, 0xa5, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 
  0x1f, 0xf8, 0xd7, 0xbd, 0xd7, 0xbd, 0x1f, 0xf8, 0x1f, 0xf8, 0x72, 0x94, 0x18, 0xc6, 0x0c, 0x63, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 
  0x1f, 0xf8, 0xaf, 0x7b, 0xaf, 0x7b, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 
  0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 
#endif
#if LV_COLOR_DEPTH == 16 && LV_COLOR_16_SWAP != 0
  /*Pixel format: Red: 5 bit, Green: 6 bit, Blue: 5 bit BUT the 2 bytes are swapped*/
  0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 
  0xf8, 0x1f, 0x7b, 0xaf, 0x7b, 0xaf, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0x5a, 0xab, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 
  0xf8, 0x1f, 0xbd, 0xd7, 0xbd, 0xd7, 0xf8, 0x1f, 0xf8, 0x1f, 0x94, 0xb2, 0xef, 0x5d, 0xb5, 0x96, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 
  0xf8, 0x1f, 0xbd, 0xd7, 0xbd, 0xd7, 0xf8, 0x1f, 0xf8, 0x1f, 0x94, 0xb2, 0xf7, 0x7e, 0xf7, 0x7e, 0xce, 0x59, 0x6b, 0x6d, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 
  0xf8, 0x1f, 0xbd, 0xd7, 0xbd, 0xd7, 0xf8, 0x1f, 0xf8, 0x1f, 0x94, 0xb2, 0xf7, 0x7e, 0xf7, 0x7e, 0xf7, 0x7e, 0xf7, 0x7e, 0xb5, 0x96, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 
  0xf8, 0x1f, 0xbd, 0xd7, 0xbd, 0xd7, 0xf8, 0x1f, 0xf8, 0x1f, 0x94, 0xb2, 0xf7, 0x7e, 0xf7, 0x7e, 0xf7, 0x7e, 0xf7, 0x7e, 0xef, 0x7d, 0xde, 0xbb, 0x6b, 0x2d, 0xf8, 0x1f, 
  0xf8, 0x1f, 0xbd, 0xd7, 0xbd, 0xd7, 0xf8, 0x1f, 0xf8, 0x1f, 0x94, 0xb2, 0xf7, 0x7e, 0xf7, 0x7e, 0xf7, 0x7e, 0xf7, 0x7e, 0xf7, 0x7e, 0xf7, 0x7e, 0xce, 0x59, 0xf8, 0x1f, 
  0xf8, 0x1f, 0xbd, 0xd7, 0xbd, 0xd7, 0xf8, 0x1f, 0xf8, 0x1f, 0x94, 0xb2, 0xf7, 0x7e, 0xf7, 0x7e, 0xf7, 0x7e, 0xf7, 0x7e, 0xef, 0x7d, 0xbd, 0xd7, 0xf8, 0x1f, 0xf8, 0x1f, 
  0xf8, 0x1f, 0xbd, 0xd7, 0xbd, 0xd7, 0xf8, 0x1f, 0xf8, 0x1f, 0x94, 0xb2, 0xf7, 0x7e, 0xf7, 0x7e, 0xef, 0x7d, 0xd6, 0x9a, 0x62, 0xec, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 
  0xf8, 0x1f, 0xbd, 0xd7, 0xbd, 0xd7, 0xf8, 0x1f, 0xf8, 0x1f, 0x94, 0xb2, 0xf7, 0x7e, 0xf7, 0x7e, 0xa5, 0x34, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 
  0xf8, 0x1f, 0xbd, 0xd7, 0xbd, 0xd7, 0xf8, 0x1f, 0xf8, 0x1f, 0x94, 0x72, 0xc6, 0x18, 0x63, 0x0c, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 
  0xf8, 0x1f, 0x7b, 0xaf, 0x7b, 0xaf, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 
  0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 
#endif
#if LV_COLOR_DEPTH == 32
  /*Pixel format: Fix 0xFF: 8 bit, Red: 8 bit, Green: 8 bit, Blue: 8 bit*/
  0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 
  0xff, 0x00, 0xff, 0xff, 0x75, 0x75, 0x75, 0xff, 0x75, 0x75, 0x75, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0x55, 0x55, 0x55, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 
  0xff, 0x00, 0xff, 0xff, 0xb7, 0xb7, 0xb7, 0xff, 0xb9, 0xb9, 0xb9, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0x93, 0x93, 0x93, 0xff, 0xe9, 0xe9, 0xe9, 0xff, 0xb0, 0xb0, 0xb0, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 
  0xff, 0x00, 0xff, 0xff, 0xb7, 0xb7, 0xb7, 0xff, 0xb9, 0xb9, 0xb9, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0x93, 0x93, 0x93, 0xff, 0xec, 0xec, 0xec, 0xff, 0xec, 0xec, 0xec, 0xff, 0xc9, 0xc9, 0xc9, 0xff, 0x6b, 0x6b, 0x6b, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 
  0xff, 0x00, 0xff, 0xff, 0xb7, 0xb7, 0xb7, 0xff, 0xb9, 0xb9, 0xb9, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0x93, 0x93, 0x93, 0xff, 0xec, 0xec, 0xec, 0xff, 0xec, 0xec, 0xec, 0xff, 0xec, 0xec, 0xec, 0xff, 0xec, 0xec, 0xec, 0xff, 0xae, 0xae, 0xae, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 
  0xff, 0x00, 0xff, 0xff, 0xb7, 0xb7, 0xb7, 0xff, 0xb9, 0xb9, 0xb9, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0x93, 0x93, 0x93, 0xff, 0xec, 0xec, 0xec, 0xff, 0xec, 0xec, 0xec, 0xff, 0xec, 0xec, 0xec, 0xff, 0xec, 0xec, 0xec, 0xff, 0xeb, 0xeb, 0xeb, 0xff, 0xd4, 0xd4, 0xd4, 0xff, 0x65, 0x65, 0x65, 0xff, 0xff, 0x00, 0xff, 0xff, 
  0xff, 0x00, 0xff, 0xff, 0xb7, 0xb7, 0xb7, 0xff, 0xb9, 0xb9, 0xb9, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0x93, 0x93, 0x93, 0xff, 0xec, 0xec, 0xec, 0xff, 0xec, 0xec, 0xec, 0xff, 0xec, 0xec, 0xec, 0xff, 0xec, 0xec, 0xec, 0xff, 0xec, 0xec, 0xec, 0xff, 0xec, 0xec, 0xec, 0xff, 0xc6, 0xc6, 0xc6, 0xff, 0xff, 0x00, 0xff, 0xff, 
  0xff, 0x00, 0xff, 0xff, 0xb7, 0xb7, 0xb7, 0xff, 0xb9, 0xb9, 0xb9, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0x93, 0x93, 0x93, 0xff, 0xec, 0xec, 0xec, 0xff, 0xec, 0xec, 0xec, 0xff, 0xec, 0xec, 0xec, 0xff, 0xec, 0xec, 0xec, 0xff, 0xea, 0xea, 0xea, 0xff, 0xb6, 0xb6, 0xb6, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 
  0xff, 0x00, 0xff, 0xff, 0xb7, 0xb7, 0xb7, 0xff, 0xb9, 0xb9, 0xb9, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0x93, 0x93, 0x93, 0xff, 0xec, 0xec, 0xec, 0xff, 0xec, 0xec, 0xec, 0xff, 0xeb, 0xeb, 0xeb, 0xff, 0xce, 0xce, 0xce, 0xff, 0x5c, 0x5c, 0x5c, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 
  0xff, 0x00, 0xff, 0xff, 0xb7, 0xb7, 0xb7, 0xff, 0xb9, 0xb9, 0xb9, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0x93, 0x93, 0x93, 0xff, 0xec, 0xec, 0xec, 0xff, 0xec, 0xec, 0xec, 0xff, 0xa2, 0xa2, 0xa2, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 
  0xff, 0x00, 0xff, 0xff, 0xb7, 0xb7, 0xb7, 0xff, 0xb9, 0xb9, 0xb9, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0x8d, 0x8d, 0x8d, 0xff, 0xc1, 0xc1, 0xc1, 0xff, 0x5f, 0x5f, 0x5f, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 
  0xff, 0x00, 0xff, 0xff, 0x75, 0x75, 0x75, 0xff, 0x75, 0x75, 0x75, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 
  0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 
#endif
};

const lv_img_dsc_t Hint_SF = {
  .header.cf = LV_IMG_CF_TRUE_COLOR_CHROMA_KEYED,
  .header.always_zero = 0,
  .header.reserved = 0,
  .header.w = 14,
  .header.h = 13,
  .data_size = 182 * LV_COLOR_SIZE / 8,
  .data = Hint_SF_map,
};