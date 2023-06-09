#ifdef LV_LVGL_H_INCLUDE_SIMPLE
#include "lvgl.h"
#else
#include "lvgl/lvgl.h"
#endif

static LV_ATTRIBUTE_LARGE_CONST const uint8_t glyph_bitmap[] = {
    /* U+4FE1 "信" */
    0x0, 0x0, 0x0, 0x80, 0x80, 0x9, 0xaa, 0x92,
    0x8a, 0xa8, 0x28, 0x0, 0x0, 0x8a, 0xa8, 0x8,
    0x6a, 0x80, 0x8c, 0x8, 0x8, 0xea, 0xc0, 0x8c,
    0x8,

    /* U+5168 "全" */
    0x0, 0x0, 0x0, 0xb, 0x0, 0x1, 0x98, 0x0,
    0xa0, 0x64, 0x3e, 0xab, 0xa0, 0x2, 0x0, 0xa,
    0xba, 0x40, 0x2, 0x0, 0x0, 0x20, 0x2, 0xab,
    0xa8, 0x0, 0x0, 0x0,

    /* U+5176 "其" */
    0x8, 0xc, 0x2, 0xea, 0xe4, 0x9, 0x5c, 0x0,
    0x95, 0xc0, 0xa, 0xac, 0x0, 0x80, 0xc0, 0x6a,
    0xaa, 0x80, 0x90, 0xd0, 0x30, 0x2, 0x40, 0x0,
    0x0,

    /* U+5220 "删" */
    0x29, 0xf4, 0x52, 0x59, 0x65, 0x25, 0x96, 0x53,
    0xae, 0x65, 0x26, 0xa6, 0x52, 0x59, 0x65, 0x25,
    0x96, 0x52, 0x69, 0x45, 0x49, 0x70, 0xc0, 0x0,
    0x0,

    /* U+53D6 "取" */
    0x0, 0x0, 0x7, 0xad, 0x0, 0x20, 0x9e, 0xc2,
    0xa8, 0x88, 0x20, 0x88, 0x82, 0xa8, 0x94, 0x20,
    0x87, 0x2, 0x6d, 0x70, 0x64, 0x98, 0xd0, 0x9,
    0x0,

    /* U+53F7 "号" */
    0xa, 0xab, 0x0, 0x80, 0x20, 0xb, 0xab, 0x0,
    0x0, 0x0, 0x2b, 0xaa, 0x90, 0x30, 0x0, 0x2,
    0xab, 0x0, 0x0, 0x30, 0x0, 0x3e, 0x0, 0x0,
    0x0,

    /* U+5907 "备" */
    0x0, 0x0, 0x0, 0xc, 0x0, 0x0, 0xba, 0xb0,
    0x6, 0xa2, 0x0, 0x0, 0xf4, 0x1, 0xb9, 0x2a,
    0x80, 0xdb, 0x64, 0x3, 0xae, 0x90, 0x8, 0x21,
    0x40, 0x3a, 0xa9, 0x0,

    /* U+5B83 "它" */
    0x0, 0x0, 0x0, 0x50, 0xa, 0xae, 0xa3, 0x0,
    0x8, 0x4c, 0x4, 0x3, 0x2e, 0x0, 0xd0, 0x0,
    0x30, 0x0, 0xc, 0x3, 0x2, 0xaa, 0x0,

    /* U+5B89 "安" */
    0x0, 0x0, 0x0, 0x5, 0x0, 0x1a, 0xba, 0x82,
    0x48, 0xc, 0x1, 0x80, 0x2, 0xba, 0xb9, 0x6,
    0x8, 0x0, 0x1a, 0x80, 0x1, 0xab, 0x42, 0x90,
    0x8,

    /* U+5BC6 "密" */
    0x0, 0x0, 0x0, 0x6, 0x0, 0x3a, 0xea, 0xd2,
    0xa, 0x51, 0x17, 0x18, 0x81, 0x39, 0x25, 0x2a,
    0xb9, 0x0, 0x82, 0x8, 0x8, 0x20, 0xc0, 0xaa,
    0xac,

    /* U+5EA6 "度" */
    0x0, 0x0, 0x0, 0x6, 0x0, 0x2a, 0xaa, 0x42,
    0x8, 0x80, 0x26, 0xee, 0x42, 0xe, 0xc0, 0x22,
    0xa9, 0x2, 0x8, 0x80, 0x60, 0x74, 0x4, 0xa4,
    0x68, 0x0, 0x0, 0x0,

    /* U+5F00 "开" */
    0x0, 0x0, 0x2, 0xba, 0xe8, 0x3, 0xc, 0x0,
    0x30, 0xc0, 0x3, 0xc, 0xa, 0xba, 0xe8, 0x3,
    0xc, 0x0, 0x60, 0xc0, 0x1c, 0xc, 0x3, 0x0,
    0xc0,

    /* U+5F3A "强" */
    0x0, 0x0, 0x2, 0xa6, 0xa8, 0x2, 0x65, 0x82,
    0xa1, 0xd0, 0x30, 0x6e, 0x83, 0x98, 0x8c, 0x1,
    0xae, 0xc0, 0x54, 0x84, 0x9, 0x1e, 0xc2, 0x89,
    0x1,

    /* U+5FFD "忽" */
    0x0, 0x0, 0x0, 0x20, 0x0, 0xa, 0xee, 0x82,
    0x59, 0x88, 0x3, 0x20, 0x80, 0xc9, 0x14, 0x1,
    0x86, 0x1, 0x13, 0x8, 0x33, 0x1, 0x61, 0x3a,
    0xa0,

    /* U+6211 "我" */
    0x0, 0x0, 0x0, 0x2, 0x20, 0x2, 0xb4, 0x98,
    0x0, 0x82, 0x0, 0x2b, 0xae, 0xa0, 0x8, 0x22,
    0x1, 0xb8, 0x88, 0x4, 0x82, 0xc0, 0x2, 0x1f,
    0x20, 0x35, 0x83, 0x80, 0x0, 0x0, 0x0,

    /* U+63A5 "接" */
    0x0, 0x0, 0x0, 0x80, 0xc0, 0x8, 0xaa, 0x96,
    0xd2, 0x24, 0x8, 0xab, 0x90, 0xd1, 0x40, 0x68,
    0xbb, 0xa0, 0x83, 0x20, 0x8, 0x1f, 0x43, 0x4a,
    0x9, 0x0, 0x0, 0x0,

    /* U+65AD "断" */
    0x8, 0x45, 0x3a, 0x69, 0x9, 0x92, 0x2, 0xb9,
    0xa9, 0x9e, 0x22, 0x2a, 0x28, 0x88, 0x82, 0x23,
    0xaa, 0x48, 0x0, 0xc3, 0x0, 0x0, 0x0,

    /* U+663E "显" */
    0xe, 0xab, 0x80, 0xd5, 0x58, 0xd, 0x56, 0x80,
    0xea, 0xb8, 0x0, 0x0, 0x1, 0x18, 0xc4, 0x19,
    0x8c, 0xc0, 0x98, 0xd4, 0x6b, 0xee, 0xa0,

    /* U+6D88 "消" */
    0x10, 0x4c, 0x40, 0xcc, 0xcc, 0x0, 0x5d, 0x42,
    0x4d, 0x5c, 0x0, 0xea, 0xc0, 0x8c, 0x8, 0x18,
    0xea, 0xc3, 0xc, 0x8, 0x30, 0xc1, 0xc0, 0x0,
    0x0,

    /* U+7259 "牙" */
    0x0, 0x0, 0x1, 0xaa, 0xe4, 0x5, 0xc, 0x0,
    0xc0, 0xc0, 0x1e, 0xae, 0x90, 0x2, 0xc0, 0x0,
    0xcc, 0x0, 0x30, 0xc0, 0x28, 0xc, 0x0, 0x2,
    0x40,

    /* U+7565 "略" */
    0x0, 0x0, 0x0, 0x1, 0x80, 0x3f, 0x3a, 0xc2,
    0xaa, 0xa0, 0x3f, 0xe, 0x2, 0xa7, 0x5d, 0x2a,
    0x6a, 0x83, 0xf2, 0x8, 0x21, 0x3a, 0xc0, 0x3,
    0x8, 0x0, 0x0, 0x0,

    /* U+7684 "的" */
    0x14, 0x20, 0x8, 0x8, 0xe, 0xba, 0xb2, 0xe,
    0x8, 0xea, 0x22, 0x20, 0x88, 0x88, 0x20, 0x23,
    0xa8, 0xc, 0x82, 0x1d, 0x0, 0x0, 0x0,

    /* U+7801 "码" */
    0x0, 0x0, 0x6, 0xa6, 0xb4, 0x14, 0x12, 0x2,
    0x2, 0x20, 0x3a, 0x22, 0x7, 0x22, 0xb8, 0xa2,
    0x0, 0x82, 0xb5, 0x68, 0x22, 0x0, 0x80, 0x0,
    0x60,

    /* U+793A "示" */
    0xa, 0xaa, 0x0, 0x0, 0x0, 0x0, 0x0, 0x2,
    0xaa, 0xa9, 0x0, 0x50, 0x0, 0x85, 0x60, 0x18,
    0x52, 0x43, 0x5, 0x8, 0x1, 0xd0, 0x0, 0x0,
    0x0,

    /* U+84DD "蓝" */
    0x6b, 0xaa, 0x80, 0x21, 0x40, 0x12, 0x20, 0x2,
    0x23, 0xa8, 0x22, 0x98, 0x1, 0x10, 0x10, 0x2b,
    0xbb, 0x2, 0x22, 0x20, 0x7b, 0xbb, 0x80,

    /* U+8BBE "设" */
    0x0, 0x0, 0x1, 0x8a, 0xb0, 0x4, 0x83, 0x0,
    0xc, 0x30, 0x2d, 0x1, 0x90, 0xce, 0xb4, 0xc,
    0x53, 0x0, 0xe3, 0xa0, 0xc, 0x2d, 0x0, 0x68,
    0x1a, 0x0, 0x0, 0x0,

    /* U+8FDE "连" */
    0x0, 0x0, 0x1, 0x2, 0x40, 0x19, 0xba, 0x80,
    0x49, 0x80, 0x24, 0xee, 0x80, 0x80, 0x80, 0x9,
    0xae, 0x90, 0x80, 0x80, 0x1d, 0x8, 0x2, 0x1a,
    0xae,

    /* U+9664 "除" */
    0x0, 0x10, 0x3b, 0x1c, 0x8, 0x88, 0x82, 0x59,
    0x1c, 0xa2, 0xb9, 0x62, 0x8, 0x8, 0xab, 0xa2,
    0x95, 0x94, 0x82, 0x22, 0x20, 0x28, 0x0, 0x0,
    0x0
};


/*---------------------
 *  GLYPH DESCRIPTION
 *--------------------*/

static const lv_font_fmt_txt_glyph_dsc_t glyph_dsc[] = {
    {.bitmap_index = 0, .adv_w = 0, .box_w = 0, .box_h = 0, .ofs_x = 0, .ofs_y = 0} /* id = 0 reserved */,
    {.bitmap_index = 0, .adv_w = 160, .box_w = 10, .box_h = 10, .ofs_x = 0, .ofs_y = -1},
    {.bitmap_index = 25, .adv_w = 160, .box_w = 10, .box_h = 11, .ofs_x = 0, .ofs_y = -2},
    {.bitmap_index = 53, .adv_w = 160, .box_w = 10, .box_h = 10, .ofs_x = 0, .ofs_y = -2},
    {.bitmap_index = 78, .adv_w = 160, .box_w = 10, .box_h = 10, .ofs_x = 0, .ofs_y = -2},
    {.bitmap_index = 103, .adv_w = 160, .box_w = 10, .box_h = 10, .ofs_x = 0, .ofs_y = -1},
    {.bitmap_index = 128, .adv_w = 160, .box_w = 10, .box_h = 10, .ofs_x = 0, .ofs_y = -2},
    {.bitmap_index = 153, .adv_w = 160, .box_w = 11, .box_h = 10, .ofs_x = 0, .ofs_y = -1},
    {.bitmap_index = 181, .adv_w = 160, .box_w = 9, .box_h = 10, .ofs_x = 1, .ofs_y = -1},
    {.bitmap_index = 204, .adv_w = 160, .box_w = 10, .box_h = 10, .ofs_x = 0, .ofs_y = -1},
    {.bitmap_index = 229, .adv_w = 160, .box_w = 10, .box_h = 10, .ofs_x = 0, .ofs_y = -1},
    {.bitmap_index = 254, .adv_w = 160, .box_w = 10, .box_h = 11, .ofs_x = 0, .ofs_y = -2},
    {.bitmap_index = 282, .adv_w = 160, .box_w = 10, .box_h = 10, .ofs_x = 0, .ofs_y = -1},
    {.bitmap_index = 307, .adv_w = 160, .box_w = 10, .box_h = 10, .ofs_x = 0, .ofs_y = -1},
    {.bitmap_index = 332, .adv_w = 160, .box_w = 10, .box_h = 10, .ofs_x = 0, .ofs_y = -1},
    {.bitmap_index = 357, .adv_w = 160, .box_w = 11, .box_h = 11, .ofs_x = 0, .ofs_y = -2},
    {.bitmap_index = 388, .adv_w = 160, .box_w = 10, .box_h = 11, .ofs_x = 0, .ofs_y = -2},
    {.bitmap_index = 416, .adv_w = 160, .box_w = 9, .box_h = 10, .ofs_x = 1, .ofs_y = -2},
    {.bitmap_index = 439, .adv_w = 160, .box_w = 10, .box_h = 9, .ofs_x = 0, .ofs_y = -1},
    {.bitmap_index = 462, .adv_w = 160, .box_w = 10, .box_h = 10, .ofs_x = 0, .ofs_y = -2},
    {.bitmap_index = 487, .adv_w = 160, .box_w = 10, .box_h = 10, .ofs_x = 0, .ofs_y = -1},
    {.bitmap_index = 512, .adv_w = 160, .box_w = 10, .box_h = 11, .ofs_x = 0, .ofs_y = -2},
    {.bitmap_index = 540, .adv_w = 160, .box_w = 9, .box_h = 10, .ofs_x = 1, .ofs_y = -2},
    {.bitmap_index = 563, .adv_w = 160, .box_w = 10, .box_h = 10, .ofs_x = 0, .ofs_y = -1},
    {.bitmap_index = 588, .adv_w = 160, .box_w = 10, .box_h = 10, .ofs_x = 0, .ofs_y = -2},
    {.bitmap_index = 613, .adv_w = 160, .box_w = 10, .box_h = 9, .ofs_x = 0, .ofs_y = -1},
    {.bitmap_index = 636, .adv_w = 160, .box_w = 10, .box_h = 11, .ofs_x = 0, .ofs_y = -2},
    {.bitmap_index = 664, .adv_w = 160, .box_w = 10, .box_h = 10, .ofs_x = 0, .ofs_y = -1},
    {.bitmap_index = 689, .adv_w = 160, .box_w = 9, .box_h = 11, .ofs_x = 1, .ofs_y = -2}
};

/*---------------------
 *  CHARACTER MAPPING
 *--------------------*/

static const uint16_t unicode_list_0[] = {
    0x0, 0x187, 0x195, 0x23f, 0x3f5, 0x416, 0x926, 0xba2,
    0xba8, 0xbe5, 0xec5, 0xf1f, 0xf59, 0x101c, 0x1230, 0x13c4,
    0x15cc, 0x165d, 0x1da7, 0x2278, 0x2584, 0x26a3, 0x2820, 0x2959,
    0x34fc, 0x3bdd, 0x3ffd, 0x4683
};

/*Collect the unicode lists and glyph_id offsets*/
static const lv_font_fmt_txt_cmap_t cmaps[] =
{
    {
        .range_start = 20449, .range_length = 18052, .glyph_id_start = 1,
        .unicode_list = unicode_list_0, .glyph_id_ofs_list = NULL, .list_length = 28, .type = LV_FONT_FMT_TXT_CMAP_SPARSE_TINY
    }
};



/*--------------------
 *  ALL CUSTOM DATA
 *--------------------*/

#if LV_VERSION_CHECK(8, 0, 0)
/*Store all the custom data of the font*/
static  lv_font_fmt_txt_glyph_cache_t cache;
static const lv_font_fmt_txt_dsc_t font_dsc = {
#else
static lv_font_fmt_txt_dsc_t font_dsc = {
#endif
    .glyph_bitmap = glyph_bitmap,
    .glyph_dsc = glyph_dsc,
    .cmaps = cmaps,
    .kern_dsc = NULL,
    .kern_scale = 0,
    .cmap_num = 1,
    .bpp = 2,
    .kern_classes = 0,
    .bitmap_format = 0,
#if LV_VERSION_CHECK(8, 0, 0)
    .cache = &cache
#endif
};


/*-----------------
 *  PUBLIC FONT
 *----------------*/

/*Initialize a public general font descriptor*/
#if LV_VERSION_CHECK(8, 0, 0)
const lv_font_t  font_china_10 = {
#else
lv_font_t  font_china_10 = {
#endif
    .get_glyph_dsc = lv_font_get_glyph_dsc_fmt_txt,    /*Function pointer to get glyph's data*/
    .get_glyph_bitmap = lv_font_get_bitmap_fmt_txt,    /*Function pointer to get glyph's bitmap*/
    .line_height = 11,          /*The maximum line height required by the font*/
    .base_line = 2,             /*Baseline measured from the bottom of the line*/
#if !(LVGL_VERSION_MAJOR == 6 && LVGL_VERSION_MINOR == 0)
    .subpx = LV_FONT_SUBPX_NONE,
#endif
#if LV_VERSION_CHECK(7, 4, 0) || LVGL_VERSION_MAJOR >= 8
    .underline_position = -1,
    .underline_thickness = 0,
#endif
    .dsc = &font_dsc           /*The custom font data. Will be accessed by `get_glyph_bitmap/dsc` */
};




