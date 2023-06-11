#pragma once
#include <stdint.h>

static const uint8_t dec_0[] = {
       2,    2,    2,    2,    2,    2,    2,    2,
       2,    2,    2,    2,    2,    2,    2,    2,
       2,    2,    2,    2,    2,    2,    2,    2,
       2,    2,    2,    2,    2,    2,    2,    2,
       2,    2,    2,    2,    2,    2,    2,    2,
       2,    2,    2,    2,    2,    2,    2,    2,
       2,    2,    2,    2,    2,    2,    2,    2,
       2,    2,    2,    2,    2,    2,    2,    2,
};

static const uint16_t enc_0[] = {
    0x0400, 0x0400, 0x0400, 0x0400, 0x0400, 0x0400, 0x0400, 0x0400,
    0x0400, 0x0400, 0x0400, 0x0400, 0x0400, 0x0400, 0x0400, 0x0400,
    0x0400, 0x0400, 0x0400, 0x0400, 0x0400, 0x0400, 0x0400, 0x0400,
    0x0400, 0x0400, 0x0400, 0x0400, 0x0400, 0x0400, 0x0400, 0x0400,
    0x0400, 0x0400, 0x0400, 0x0400, 0x0400, 0x0400, 0x0400, 0x0400,
    0x0400, 0x0400, 0x0400, 0x0400, 0x0400, 0x0400, 0x0400, 0x0400,
    0x0400, 0x0400, 0x0400, 0x0400, 0x0400, 0x0400, 0x0400, 0x0400,
    0x0400, 0x0400, 0x0400, 0x0400, 0x0400, 0x0400, 0x0400, 0x0400,
};

static const uint8_t dec_1[] = {
       2,    2,    2,    2,    2,    2,    2,    2,
       2,    2,    2,    2,    2,    2,    2,    3,
       2,    2,    2,    2,    2,    2,    3,    3,
       2,    2,    2,    2,    2,    3,    3,    3,
       2,    2,    2,    2,    3,    3,    3,    3,
       2,    2,    2,    3,    3,    3,    3,    3,
       2,    2,    3,    3,    3,    3,    3,    3,
       2,    3,    3,    3,    3,    3,    3,    3,
};

static const uint16_t enc_1[] = {
    0x0400, 0x0400, 0x0400, 0x0400, 0x0400, 0x0400, 0x0400, 0x0400,
    0x0400, 0x0400, 0x0400, 0x0400, 0x0400, 0x0400, 0x0400, 0x0400,
    0x0400, 0x0400, 0x0400, 0x0400, 0x0400, 0x0400, 0x0400, 0x0400,
    0x0400, 0x0400, 0x0400, 0x0400, 0x0400, 0x0400, 0x0400, 0x0400,
    0x0400, 0x0400, 0x0400, 0x0400, 0x0d55, 0x0d55, 0x0d55, 0x0d55,
    0x0d55, 0x0d55, 0x0d55, 0x0d55, 0x0d55, 0x0d55, 0x0d55, 0x0d55,
    0x0d55, 0x0d55, 0x0d55, 0x0d55, 0x0d55, 0x0d55, 0x0d55, 0x0d55,
    0x0d55, 0x0d55, 0x0d55, 0x0d55, 0x0d55, 0x0d55, 0x0d55, 0x0d55,
};

static const uint8_t dec_2[] = {
       2,    2,    2,    2,    3,    3,    3,    3,
       2,    2,    2,    3,    3,    3,    3,    3,
       2,    2,    2,    3,    3,    3,    3,    3,
       2,    3,    3,    3,    3,    3,    3,    3,
       3,    3,    3,    3,    3,    3,    3,    3,
       3,    3,    3,    3,    3,    3,    3,    3,
       3,    3,    3,    3,    3,    3,    3,    3,
       3,    3,    3,    3,    3,    3,    3,    3,
};

static const uint16_t enc_2[] = {
    0x0400, 0x0400, 0x0400, 0x0400, 0x0400, 0x0400, 0x0400, 0x0400,
    0x0400, 0x0400, 0x0d55, 0x0d55, 0x0400, 0x0d55, 0x0d55, 0x0d55,
    0x0d55, 0x0d55, 0x0d55, 0x0d55, 0x0d55, 0x0d55, 0x0d55, 0x0d55,
    0x0d55, 0x0d55, 0x0d55, 0x0d55, 0x0d55, 0x0d55, 0x0d55, 0x0d55,
    0x0d55, 0x0d55, 0x0d55, 0x0d55, 0x0d55, 0x0d55, 0x0d55, 0x0d55,
    0x0d55, 0x0d55, 0x0d55, 0x0d55, 0x0d55, 0x0d55, 0x0d55, 0x0d55,
    0x0d55, 0x0d55, 0x0d55, 0x0d55, 0x0d55, 0x0d55, 0x0d55, 0x0d55,
    0x0d55, 0x0d55, 0x0d55, 0x0d55, 0x0d55, 0x0d55, 0x0d55, 0x0d55,
};

static const uint8_t dec_3[] = {
       2,    2,    2,    2,    3,    3,    3,    3,
       2,    2,    2,    3,    3,    3,    3,    3,
       2,    2,    2,    3,    3,    3,    3,    4,
       2,    3,    3,    3,    3,    3,    4,    4,
       3,    3,    3,    3,    3,    4,    4,    4,
       3,    3,    3,    3,    4,    4,    4,    4,
       3,    3,    3,    4,    4,    4,    4,    4,
       3,    3,    4,    4,    4,    4,    4,    4,
};

static const uint16_t enc_3[] = {
    0x0400, 0x0400, 0x0400, 0x0400, 0x0400, 0x0400, 0x0400, 0x0400,
    0x0400, 0x0400, 0x0d55, 0x0d55, 0x0400, 0x0d55, 0x0d55, 0x0d55,
    0x0d55, 0x0d55, 0x0d55, 0x0d55, 0x0d55, 0x0d55, 0x0d55, 0x0d55,
    0x0d55, 0x0d55, 0x0d55, 0x0d55, 0x0d55, 0x0d55, 0x0d55, 0x0d55,
    0x0d55, 0x0d55, 0x0d55, 0x0d55, 0x0d55, 0x0d55, 0x0d55, 0x0d55,
    0x0d55, 0x0d55, 0x0d55, 0x0c00, 0x0c00, 0x0c00, 0x0c00, 0x0c00,
    0x0c00, 0x0c00, 0x0c00, 0x0c00, 0x0c00, 0x0c00, 0x0c00, 0x0c00,
    0x0c00, 0x0c00, 0x0c00, 0x0c00, 0x0c00, 0x0c00, 0x0c00, 0x0c00,
};

static const uint8_t dec_4[] = {
       2,    2,    2,    2,    4,    4,    4,    4,
       2,    2,    2,    3,    4,    4,    4,    4,
       2,    2,    2,    4,    4,    4,    4,    4,
       2,    3,    4,    4,    4,    4,    4,    4,
       4,    4,    4,    4,    4,    4,    4,    4,
       4,    4,    4,    4,    4,    4,    4,    4,
       4,    4,    4,    4,    4,    4,    4,    4,
       4,    4,    4,    4,    4,    4,    4,    4,
};

static const uint16_t enc_4[] = {
    0x0400, 0x0400, 0x0400, 0x0400, 0x0400, 0x0400, 0x0400, 0x0400,
    0x0400, 0x0400, 0x0c00, 0x0d55, 0x0400, 0x0d55, 0x0c00, 0x0c00,
    0x0c00, 0x0c00, 0x0c00, 0x0c00, 0x0c00, 0x0c00, 0x0c00, 0x0c00,
    0x0c00, 0x0c00, 0x0c00, 0x0c00, 0x0c00, 0x0c00, 0x0c00, 0x0c00,
    0x0c00, 0x0c00, 0x0c00, 0x0c00, 0x0c00, 0x0c00, 0x0c00, 0x0c00,
    0x0c00, 0x0c00, 0x0c00, 0x0c00, 0x0c00, 0x0c00, 0x0c00, 0x0c00,
    0x0c00, 0x0c00, 0x0c00, 0x0c00, 0x0c00, 0x0c00, 0x0c00, 0x0c00,
    0x0c00, 0x0c00, 0x0c00, 0x0c00, 0x0c00, 0x0c00, 0x0c00, 0x0c00,
};

static const uint8_t dec_5[] = {
       2,    2,    2,    3,    6,    6,    6,    6,
       2,    2,    2,    4,    6,    6,    6,    6,
       2,    2,    3,    6,    6,    6,    6,    6,
       3,    4,    6,    6,    6,    6,    6,    6,
       6,    6,    6,    6,    6,    6,    6,    6,
       6,    6,    6,    6,    6,    6,    6,    6,
       6,    6,    6,    6,    6,    6,    6,    6,
       6,    6,    6,    6,    6,    6,    6,    6,
};

static const uint16_t enc_5[] = {
    0x0400, 0x0400, 0x0400, 0x0400, 0x0400, 0x0400, 0x0d55, 0x0400,
    0x0400, 0x0d55, 0x1555, 0x0c00, 0x0d55, 0x0c00, 0x1555, 0x1555,
    0x1555, 0x1555, 0x1555, 0x1555, 0x1555, 0x1555, 0x1555, 0x1555,
    0x1555, 0x1555, 0x1555, 0x1555, 0x1555, 0x1555, 0x1555, 0x1555,
    0x1555, 0x1555, 0x1555, 0x1555, 0x1555, 0x1555, 0x1555, 0x1555,
    0x1555, 0x1555, 0x1555, 0x1555, 0x1555, 0x1555, 0x1555, 0x1555,
    0x1555, 0x1555, 0x1555, 0x1555, 0x1555, 0x1555, 0x1555, 0x1555,
    0x1555, 0x1555, 0x1555, 0x1555, 0x1555, 0x1555, 0x1555, 0x1555,
};

static const uint8_t dec_6[] = {
       2,    2,    2,    4,    8,    8,    8,    8,
       2,    2,    2,    5,    8,    8,    8,    8,
       2,    2,    4,    8,    8,    8,    8,    8,
       4,    5,    8,    8,    8,    8,    8,    8,
       8,    8,    8,    8,    8,    8,    8,    8,
       8,    8,    8,    8,    8,    8,    8,    8,
       8,    8,    8,    8,    8,    8,    8,    8,
       8,    8,    8,    8,    8,    8,    8,    8,
};

static const uint16_t enc_6[] = {
    0x0400, 0x0400, 0x0400, 0x0400, 0x0400, 0x0400, 0x0c00, 0x0400,
    0x0400, 0x0c00, 0x1400, 0x1666, 0x0c00, 0x1666, 0x1400, 0x1400,
    0x1400, 0x1400, 0x1400, 0x1400, 0x1400, 0x1400, 0x1400, 0x1400,
    0x1400, 0x1400, 0x1400, 0x1400, 0x1400, 0x1400, 0x1400, 0x1400,
    0x1400, 0x1400, 0x1400, 0x1400, 0x1400, 0x1400, 0x1400, 0x1400,
    0x1400, 0x1400, 0x1400, 0x1400, 0x1400, 0x1400, 0x1400, 0x1400,
    0x1400, 0x1400, 0x1400, 0x1400, 0x1400, 0x1400, 0x1400, 0x1400,
    0x1400, 0x1400, 0x1400, 0x1400, 0x1400, 0x1400, 0x1400, 0x1400,
};

static const uint8_t dec_7[] = {
       2,    2,    2,    5,   10,   10,   10,   10,
       2,    2,    3,    7,   10,   10,   10,   10,
       2,    3,    6,   10,   10,   10,   10,   10,
       5,    7,   10,   10,   10,   10,   10,   10,
      10,   10,   10,   10,   10,   10,   10,   10,
      10,   10,   10,   10,   10,   10,   10,   10,
      10,   10,   10,   10,   10,   10,   10,   10,
      10,   10,   10,   10,   10,   10,   10,   10,
};

static const uint16_t enc_7[] = {
    0x0400, 0x0400, 0x0400, 0x0400, 0x0400, 0x0400, 0x1666, 0x0d55,
    0x0d55, 0x1666, 0x1e66, 0x1492, 0x1555, 0x1492, 0x1e66, 0x1e66,
    0x1e66, 0x1e66, 0x1e66, 0x1e66, 0x1e66, 0x1e66, 0x1e66, 0x1e66,
    0x1e66, 0x1e66, 0x1e66, 0x1e66, 0x1e66, 0x1e66, 0x1e66, 0x1e66,
    0x1e66, 0x1e66, 0x1e66, 0x1e66, 0x1e66, 0x1e66, 0x1e66, 0x1e66,
    0x1e66, 0x1e66, 0x1e66, 0x1e66, 0x1e66, 0x1e66, 0x1e66, 0x1e66,
    0x1e66, 0x1e66, 0x1e66, 0x1e66, 0x1e66, 0x1e66, 0x1e66, 0x1e66,
    0x1e66, 0x1e66, 0x1e66, 0x1e66, 0x1e66, 0x1e66, 0x1e66, 0x1e66,
};

static const uint8_t dec_8[] = {
       2,    2,    3,    6,   12,   12,   12,   12,
       2,    3,    3,    8,   12,   12,   12,   12,
       3,    3,    7,   12,   12,   12,   12,   12,
       6,    8,   12,   12,   12,   12,   12,   12,
      12,   12,   12,   12,   12,   12,   12,   12,
      12,   12,   12,   12,   12,   12,   12,   12,
      12,   12,   12,   12,   12,   12,   12,   12,
      12,   12,   12,   12,   12,   12,   12,   12,
};

static const uint16_t enc_8[] = {
    0x0400, 0x0400, 0x0400, 0x0d55, 0x0d55, 0x0d55, 0x1555, 0x0d55,
    0x0d55, 0x1555, 0x1d55, 0x1400, 0x1492, 0x1400, 0x1d55, 0x1d55,
    0x1d55, 0x1d55, 0x1d55, 0x1d55, 0x1d55, 0x1d55, 0x1d55, 0x1d55,
    0x1d55, 0x1d55, 0x1d55, 0x1d55, 0x1d55, 0x1d55, 0x1d55, 0x1d55,
    0x1d55, 0x1d55, 0x1d55, 0x1d55, 0x1d55, 0x1d55, 0x1d55, 0x1d55,
    0x1d55, 0x1d55, 0x1d55, 0x1d55, 0x1d55, 0x1d55, 0x1d55, 0x1d55,
    0x1d55, 0x1d55, 0x1d55, 0x1d55, 0x1d55, 0x1d55, 0x1d55, 0x1d55,
    0x1d55, 0x1d55, 0x1d55, 0x1d55, 0x1d55, 0x1d55, 0x1d55, 0x1d55,
};

static const uint8_t dec_9[] = {
       2,    3,    3,    7,   14,   14,   14,   14,
       3,    3,    4,    9,   14,   14,   14,   14,
       3,    4,    8,   14,   14,   14,   14,   14,
       7,    9,   14,   14,   14,   14,   14,   14,
      14,   14,   14,   14,   14,   14,   14,   14,
      14,   14,   14,   14,   14,   14,   14,   14,
      14,   14,   14,   14,   14,   14,   14,   14,
      14,   14,   14,   14,   14,   14,   14,   14,
};

static const uint16_t enc_9[] = {
    0x0400, 0x0d55, 0x0d55, 0x0d55, 0x0d55, 0x0d55, 0x1492, 0x0c00,
    0x0c00, 0x1492, 0x1c92, 0x1f1c, 0x1400, 0x1f1c, 0x1c92, 0x1c92,
    0x1c92, 0x1c92, 0x1c92, 0x1c92, 0x1c92, 0x1c92, 0x1c92, 0x1c92,
    0x1c92, 0x1c92, 0x1c92, 0x1c92, 0x1c92, 0x1c92, 0x1c92, 0x1c92,
    0x1c92, 0x1c92, 0x1c92, 0x1c92, 0x1c92, 0x1c92, 0x1c92, 0x1c92,
    0x1c92, 0x1c92, 0x1c92, 0x1c92, 0x1c92, 0x1c92, 0x1c92, 0x1c92,
    0x1c92, 0x1c92, 0x1c92, 0x1c92, 0x1c92, 0x1c92, 0x1c92, 0x1c92,
    0x1c92, 0x1c92, 0x1c92, 0x1c92, 0x1c92, 0x1c92, 0x1c92, 0x1c92,
};

static const uint8_t dec_10[] = {
       3,    3,    4,    8,   16,   16,   16,   16,
       3,    3,    4,   11,   16,   16,   16,   16,
       4,    4,    9,   16,   16,   16,   16,   16,
       8,   11,   16,   16,   16,   16,   16,   16,
      16,   16,   16,   16,   16,   16,   16,   16,
      16,   16,   16,   16,   16,   16,   16,   16,
      16,   16,   16,   16,   16,   16,   16,   16,
      16,   16,   16,   16,   16,   16,   16,   16,
};

static const uint16_t enc_10[] = {
    0x0d55, 0x0d55, 0x0d55, 0x0c00, 0x0d55, 0x0c00, 0x1400, 0x0c00,
    0x0c00, 0x1400, 0x1c00, 0x1dd1, 0x1f1c, 0x1dd1, 0x1c00, 0x1c00,
    0x1c00, 0x1c00, 0x1c00, 0x1c00, 0x1c00, 0x1c00, 0x1c00, 0x1c00,
    0x1c00, 0x1c00, 0x1c00, 0x1c00, 0x1c00, 0x1c00, 0x1c00, 0x1c00,
    0x1c00, 0x1c00, 0x1c00, 0x1c00, 0x1c00, 0x1c00, 0x1c00, 0x1c00,
    0x1c00, 0x1c00, 0x1c00, 0x1c00, 0x1c00, 0x1c00, 0x1c00, 0x1c00,
    0x1c00, 0x1c00, 0x1c00, 0x1c00, 0x1c00, 0x1c00, 0x1c00, 0x1c00,
    0x1c00, 0x1c00, 0x1c00, 0x1c00, 0x1c00, 0x1c00, 0x1c00, 0x1c00,
};

static const uint8_t dec_11[] = {
       3,    3,    4,    8,   18,   18,   18,   18,
       3,    4,    5,   12,   18,   18,   18,   18,
       4,    5,   10,   18,   18,   18,   18,   18,
       8,   12,   18,   18,   18,   18,   18,   18,
      18,   18,   18,   18,   18,   18,   18,   18,
      18,   18,   18,   18,   18,   18,   18,   18,
      18,   18,   18,   18,   18,   18,   18,   18,
      18,   18,   18,   18,   18,   18,   18,   18,
};

static const uint16_t enc_11[] = {
    0x0d55, 0x0d55, 0x0d55, 0x0c00, 0x0c00, 0x0c00, 0x1400, 0x1666,
    0x1666, 0x1400, 0x271c, 0x1d55, 0x1e66, 0x1d55, 0x271c, 0x271c,
    0x271c, 0x271c, 0x271c, 0x271c, 0x271c, 0x271c, 0x271c, 0x271c,
    0x271c, 0x271c, 0x271c, 0x271c, 0x271c, 0x271c, 0x271c, 0x271c,
    0x271c, 0x271c, 0x271c, 0x271c, 0x271c, 0x271c, 0x271c, 0x271c,
    0x271c, 0x271c, 0x271c, 0x271c, 0x271c, 0x271c, 0x271c, 0x271c,
    0x271c, 0x271c, 0x271c, 0x271c, 0x271c, 0x271c, 0x271c, 0x271c,
    0x271c, 0x271c, 0x271c, 0x271c, 0x271c, 0x271c, 0x271c, 0x271c,
};

static const uint8_t dec_12[] = {
       3,    4,    5,    9,   20,   20,   20,   20,
       4,    4,    5,   13,   20,   20,   20,   20,
       5,    5,   11,   20,   20,   20,   20,   20,
       9,   13,   20,   20,   20,   20,   20,   20,
      20,   20,   20,   20,   20,   20,   20,   20,
      20,   20,   20,   20,   20,   20,   20,   20,
      20,   20,   20,   20,   20,   20,   20,   20,
      20,   20,   20,   20,   20,   20,   20,   20,
};

static const uint16_t enc_12[] = {
    0x0d55, 0x0c00, 0x0c00, 0x1666, 0x0c00, 0x1666, 0x1f1c, 0x1666,
    0x1666, 0x1f1c, 0x2666, 0x1cec, 0x1dd1, 0x1cec, 0x2666, 0x2666,
    0x2666, 0x2666, 0x2666, 0x2666, 0x2666, 0x2666, 0x2666, 0x2666,
    0x2666, 0x2666, 0x2666, 0x2666, 0x2666, 0x2666, 0x2666, 0x2666,
    0x2666, 0x2666, 0x2666, 0x2666, 0x2666, 0x2666, 0x2666, 0x2666,
    0x2666, 0x2666, 0x2666, 0x2666, 0x2666, 0x2666, 0x2666, 0x2666,
    0x2666, 0x2666, 0x2666, 0x2666, 0x2666, 0x2666, 0x2666, 0x2666,
    0x2666, 0x2666, 0x2666, 0x2666, 0x2666, 0x2666, 0x2666, 0x2666,
};

static const uint8_t dec_13[] = {
       4,    4,    6,   11,   24,   24,   24,   24,
       4,    5,    6,   16,   24,   24,   24,   24,
       6,    6,   13,   24,   24,   24,   24,   24,
      11,   16,   24,   24,   24,   24,   24,   24,
      24,   24,   24,   24,   24,   24,   24,   24,
      24,   24,   24,   24,   24,   24,   24,   24,
      24,   24,   24,   24,   24,   24,   24,   24,
      24,   24,   24,   24,   24,   24,   24,   24,
};

static const uint16_t enc_13[] = {
    0x0c00, 0x0c00, 0x0c00, 0x1555, 0x1666, 0x1555, 0x1dd1, 0x1555,
    0x1555, 0x1dd1, 0x2555, 0x1c00, 0x1cec, 0x1c00, 0x2555, 0x2555,
    0x2555, 0x2555, 0x2555, 0x2555, 0x2555, 0x2555, 0x2555, 0x2555,
    0x2555, 0x2555, 0x2555, 0x2555, 0x2555, 0x2555, 0x2555, 0x2555,
    0x2555, 0x2555, 0x2555, 0x2555, 0x2555, 0x2555, 0x2555, 0x2555,
    0x2555, 0x2555, 0x2555, 0x2555, 0x2555, 0x2555, 0x2555, 0x2555,
    0x2555, 0x2555, 0x2555, 0x2555, 0x2555, 0x2555, 0x2555, 0x2555,
    0x2555, 0x2555, 0x2555, 0x2555, 0x2555, 0x2555, 0x2555, 0x2555,
};

static const uint8_t dec_14[] = {
       5,    5,    7,   13,   28,   28,   28,   28,
       5,    6,    7,   18,   28,   28,   28,   28,
       7,    7,   16,   28,   28,   28,   28,   28,
      13,   18,   28,   28,   28,   28,   28,   28,
      28,   28,   28,   28,   28,   28,   28,   28,
      28,   28,   28,   28,   28,   28,   28,   28,
      28,   28,   28,   28,   28,   28,   28,   28,
      28,   28,   28,   28,   28,   28,   28,   28,
};

static const uint16_t enc_14[] = {
    0x1666, 0x1666, 0x1666, 0x1492, 0x1555, 0x1492, 0x1cec, 0x1492,
    0x1492, 0x1cec, 0x2492, 0x271c, 0x1c00, 0x271c, 0x2492, 0x2492,
    0x2492, 0x2492, 0x2492, 0x2492, 0x2492, 0x2492, 0x2492, 0x2492,
    0x2492, 0x2492, 0x2492, 0x2492, 0x2492, 0x2492, 0x2492, 0x2492,
    0x2492, 0x2492, 0x2492, 0x2492, 0x2492, 0x2492, 0x2492, 0x2492,
    0x2492, 0x2492, 0x2492, 0x2492, 0x2492, 0x2492, 0x2492, 0x2492,
    0x2492, 0x2492, 0x2492, 0x2492, 0x2492, 0x2492, 0x2492, 0x2492,
    0x2492, 0x2492, 0x2492, 0x2492, 0x2492, 0x2492, 0x2492, 0x2492,
};

static const uint8_t dec_15[] = {
       5,    6,    8,   15,   32,   32,   32,   32,
       6,    7,    8,   21,   32,   32,   32,   32,
       8,    8,   18,   32,   32,   32,   32,   32,
      15,   21,   32,   32,   32,   32,   32,   32,
      32,   32,   32,   32,   32,   32,   32,   32,
      32,   32,   32,   32,   32,   32,   32,   32,
      32,   32,   32,   32,   32,   32,   32,   32,
      32,   32,   32,   32,   32,   32,   32,   32,
};

static const uint16_t enc_15[] = {
    0x1666, 0x1555, 0x1555, 0x1400, 0x1492, 0x1400, 0x1c44, 0x1400,
    0x1400, 0x1c44, 0x2400, 0x2618, 0x271c, 0x2618, 0x2400, 0x2400,
    0x2400, 0x2400, 0x2400, 0x2400, 0x2400, 0x2400, 0x2400, 0x2400,
    0x2400, 0x2400, 0x2400, 0x2400, 0x2400, 0x2400, 0x2400, 0x2400,
    0x2400, 0x2400, 0x2400, 0x2400, 0x2400, 0x2400, 0x2400, 0x2400,
    0x2400, 0x2400, 0x2400, 0x2400, 0x2400, 0x2400, 0x2400, 0x2400,
    0x2400, 0x2400, 0x2400, 0x2400, 0x2400, 0x2400, 0x2400, 0x2400,
    0x2400, 0x2400, 0x2400, 0x2400, 0x2400, 0x2400, 0x2400, 0x2400,
};

static const uint8_t dec_16[] = {
       6,    6,    9,   17,   36,   36,   36,   36,
       6,    8,    9,   24,   36,   36,   36,   36,
       9,    9,   20,   36,   36,   36,   36,   36,
      17,   24,   36,   36,   36,   36,   36,   36,
      36,   36,   36,   36,   36,   36,   36,   36,
      36,   36,   36,   36,   36,   36,   36,   36,
      36,   36,   36,   36,   36,   36,   36,   36,
      36,   36,   36,   36,   36,   36,   36,   36,
};

static const uint16_t enc_16[] = {
    0x1555, 0x1555, 0x1555, 0x1f1c, 0x1400, 0x1f1c, 0x2788, 0x1f1c,
    0x1f1c, 0x2788, 0x2f1c, 0x2555, 0x2666, 0x2555, 0x2f1c, 0x2f1c,
    0x2f1c, 0x2f1c, 0x2f1c, 0x2f1c, 0x2f1c, 0x2f1c, 0x2f1c, 0x2f1c,
    0x2f1c, 0x2f1c, 0x2f1c, 0x2f1c, 0x2f1c, 0x2f1c, 0x2f1c, 0x2f1c,
    0x2f1c, 0x2f1c, 0x2f1c, 0x2f1c, 0x2f1c, 0x2f1c, 0x2f1c, 0x2f1c,
    0x2f1c, 0x2f1c, 0x2f1c, 0x2f1c, 0x2f1c, 0x2f1c, 0x2f1c, 0x2f1c,
    0x2f1c, 0x2f1c, 0x2f1c, 0x2f1c, 0x2f1c, 0x2f1c, 0x2f1c, 0x2f1c,
    0x2f1c, 0x2f1c, 0x2f1c, 0x2f1c, 0x2f1c, 0x2f1c, 0x2f1c, 0x2f1c,
};

static const uint8_t dec_17[] = {
       7,    7,   10,   19,   40,   40,   40,   40,
       7,    8,   10,   26,   40,   40,   40,   40,
      10,   10,   22,   40,   40,   40,   40,   40,
      19,   26,   40,   40,   40,   40,   40,   40,
      40,   40,   40,   40,   40,   40,   40,   40,
      40,   40,   40,   40,   40,   40,   40,   40,
      40,   40,   40,   40,   40,   40,   40,   40,
      40,   40,   40,   40,   40,   40,   40,   40,
};

static const uint16_t enc_17[] = {
    0x1492, 0x1492, 0x1492, 0x1e66, 0x1400, 0x1e66, 0x26bd, 0x1e66,
    0x1e66, 0x26bd, 0x2e66, 0x24ec, 0x25d1, 0x24ec, 0x2e66, 0x2e66,
    0x2e66, 0x2e66, 0x2e66, 0x2e66, 0x2e66, 0x2e66, 0x2e66, 0x2e66,
    0x2e66, 0x2e66, 0x2e66, 0x2e66, 0x2e66, 0x2e66, 0x2e66, 0x2e66,
    0x2e66, 0x2e66, 0x2e66, 0x2e66, 0x2e66, 0x2e66, 0x2e66, 0x2e66,
    0x2e66, 0x2e66, 0x2e66, 0x2e66, 0x2e66, 0x2e66, 0x2e66, 0x2e66,
    0x2e66, 0x2e66, 0x2e66, 0x2e66, 0x2e66, 0x2e66, 0x2e66, 0x2e66,
    0x2e66, 0x2e66, 0x2e66, 0x2e66, 0x2e66, 0x2e66, 0x2e66, 0x2e66,
};

static const uint8_t dec_18[] = {
       7,    8,   11,   21,   44,   44,   44,   44,
       8,    9,   11,   29,   44,   44,   44,   44,
      11,   11,   25,   44,   44,   44,   44,   44,
      21,   29,   44,   44,   44,   44,   44,   44,
      44,   44,   44,   44,   44,   44,   44,   44,
      44,   44,   44,   44,   44,   44,   44,   44,
      44,   44,   44,   44,   44,   44,   44,   44,
      44,   44,   44,   44,   44,   44,   44,   44,
};

static const uint16_t enc_18[] = {
    0x1492, 0x1400, 0x1400, 0x1dd1, 0x1f1c, 0x1dd1, 0x2618, 0x1dd1,
    0x1dd1, 0x2618, 0x2dd1, 0x246a, 0x251f, 0x246a, 0x2dd1, 0x2dd1,
    0x2dd1, 0x2dd1, 0x2dd1, 0x2dd1, 0x2dd1, 0x2dd1, 0x2dd1, 0x2dd1,
    0x2dd1, 0x2dd1, 0x2dd1, 0x2dd1, 0x2dd1, 0x2dd1, 0x2dd1, 0x2dd1,
    0x2dd1, 0x2dd1, 0x2dd1, 0x2dd1, 0x2dd1, 0x2dd1, 0x2dd1, 0x2dd1,
    0x2dd1, 0x2dd1, 0x2dd1, 0x2dd1, 0x2dd1, 0x2dd1, 0x2dd1, 0x2dd1,
    0x2dd1, 0x2dd1, 0x2dd1, 0x2dd1, 0x2dd1, 0x2dd1, 0x2dd1, 0x2dd1,
    0x2dd1, 0x2dd1, 0x2dd1, 0x2dd1, 0x2dd1, 0x2dd1, 0x2dd1, 0x2dd1,
};

static const uint8_t dec_19[] = {
       8,    9,   12,   23,   48,   48,   48,   48,
       9,   10,   12,   32,   48,   48,   48,   48,
      12,   12,   27,   48,   48,   48,   48,   48,
      23,   32,   48,   48,   48,   48,   48,   48,
      48,   48,   48,   48,   48,   48,   48,   48,
      48,   48,   48,   48,   48,   48,   48,   48,
      48,   48,   48,   48,   48,   48,   48,   48,
      48,   48,   48,   48,   48,   48,   48,   48,
};

static const uint16_t enc_19[] = {
    0x1400, 0x1f1c, 0x1f1c, 0x1d55, 0x1e66, 0x1d55, 0x2591, 0x1d55,
    0x1d55, 0x2591, 0x2d55, 0x2400, 0x24be, 0x2400, 0x2d55, 0x2d55,
    0x2d55, 0x2d55, 0x2d55, 0x2d55, 0x2d55, 0x2d55, 0x2d55, 0x2d55,
    0x2d55, 0x2d55, 0x2d55, 0x2d55, 0x2d55, 0x2d55, 0x2d55, 0x2d55,
    0x2d55, 0x2d55, 0x2d55, 0x2d55, 0x2d55, 0x2d55, 0x2d55, 0x2d55,
    0x2d55, 0x2d55, 0x2d55, 0x2d55, 0x2d55, 0x2d55, 0x2d55, 0x2d55,
    0x2d55, 0x2d55, 0x2d55, 0x2d55, 0x2d55, 0x2d55, 0x2d55, 0x2d55,
    0x2d55, 0x2d55, 0x2d55, 0x2d55, 0x2d55, 0x2d55, 0x2d55, 0x2d55,
};

static const uint8_t dec_20[] = {
       9,    9,   12,   24,   51,   51,   51,   51,
       9,   11,   14,   34,   51,   51,   51,   51,
      12,   14,   29,   51,   51,   51,   51,   51,
      24,   34,   51,   51,   51,   51,   51,   51,
      51,   51,   51,   51,   51,   51,   51,   51,
      51,   51,   51,   51,   51,   51,   51,   51,
      51,   51,   51,   51,   51,   51,   51,   51,
      51,   51,   51,   51,   51,   51,   51,   51,
};

static const uint16_t enc_20[] = {
    0x1f1c, 0x1f1c, 0x1f1c, 0x1d55, 0x1dd1, 0x1d55, 0x2555, 0x1c92,
    0x1c92, 0x2555, 0x2d05, 0x2f88, 0x246a, 0x2f88, 0x2d05, 0x2d05,
    0x2d05, 0x2d05, 0x2d05, 0x2d05, 0x2d05, 0x2d05, 0x2d05, 0x2d05,
    0x2d05, 0x2d05, 0x2d05, 0x2d05, 0x2d05, 0x2d05, 0x2d05, 0x2d05,
    0x2d05, 0x2d05, 0x2d05, 0x2d05, 0x2d05, 0x2d05, 0x2d05, 0x2d05,
    0x2d05, 0x2d05, 0x2d05, 0x2d05, 0x2d05, 0x2d05, 0x2d05, 0x2d05,
    0x2d05, 0x2d05, 0x2d05, 0x2d05, 0x2d05, 0x2d05, 0x2d05, 0x2d05,
    0x2d05, 0x2d05, 0x2d05, 0x2d05, 0x2d05, 0x2d05, 0x2d05, 0x2d05,
};

static const uint8_t dec_21[] = {
      10,   10,   13,   26,   55,   55,   55,   55,
      10,   12,   15,   37,   55,   55,   55,   55,
      13,   15,   31,   55,   55,   55,   55,   55,
      26,   37,   55,   55,   55,   55,   55,   55,
      55,   55,   55,   55,   55,   55,   55,   55,
      55,   55,   55,   55,   55,   55,   55,   55,
      55,   55,   55,   55,   55,   55,   55,   55,
      55,   55,   55,   55,   55,   55,   55,   55,
};

static const uint16_t enc_21[] = {
    0x1e66, 0x1e66, 0x1e66, 0x1cec, 0x1d55, 0x1cec, 0x24ec, 0x1c44,
    0x1c44, 0x24ec, 0x2ca8, 0x2eeb, 0x2421, 0x2eeb, 0x2ca8, 0x2ca8,
    0x2ca8, 0x2ca8, 0x2ca8, 0x2ca8, 0x2ca8, 0x2ca8, 0x2ca8, 0x2ca8,
    0x2ca8, 0x2ca8, 0x2ca8, 0x2ca8, 0x2ca8, 0x2ca8, 0x2ca8, 0x2ca8,
    0x2ca8, 0x2ca8, 0x2ca8, 0x2ca8, 0x2ca8, 0x2ca8, 0x2ca8, 0x2ca8,
    0x2ca8, 0x2ca8, 0x2ca8, 0x2ca8, 0x2ca8, 0x2ca8, 0x2ca8, 0x2ca8,
    0x2ca8, 0x2ca8, 0x2ca8, 0x2ca8, 0x2ca8, 0x2ca8, 0x2ca8, 0x2ca8,
    0x2ca8, 0x2ca8, 0x2ca8, 0x2ca8, 0x2ca8, 0x2ca8, 0x2ca8, 0x2ca8,
};

static const uint8_t dec_22[] = {
      10,   11,   14,   28,   59,   59,   59,   59,
      11,   13,   16,   40,   59,   59,   59,   59,
      14,   16,   34,   59,   59,   59,   59,   59,
      28,   40,   59,   59,   59,   59,   59,   59,
      59,   59,   59,   59,   59,   59,   59,   59,
      59,   59,   59,   59,   59,   59,   59,   59,
      59,   59,   59,   59,   59,   59,   59,   59,
      59,   59,   59,   59,   59,   59,   59,   59,
};

static const uint16_t enc_22[] = {
    0x1e66, 0x1dd1, 0x1dd1, 0x1c92, 0x1cec, 0x1c92, 0x2492, 0x1c00,
    0x1c00, 0x2492, 0x2c57, 0x2e66, 0x2f88, 0x2e66, 0x2c57, 0x2c57,
    0x2c57, 0x2c57, 0x2c57, 0x2c57, 0x2c57, 0x2c57, 0x2c57, 0x2c57,
    0x2c57, 0x2c57, 0x2c57, 0x2c57, 0x2c57, 0x2c57, 0x2c57, 0x2c57,
    0x2c57, 0x2c57, 0x2c57, 0x2c57, 0x2c57, 0x2c57, 0x2c57, 0x2c57,
    0x2c57, 0x2c57, 0x2c57, 0x2c57, 0x2c57, 0x2c57, 0x2c57, 0x2c57,
    0x2c57, 0x2c57, 0x2c57, 0x2c57, 0x2c57, 0x2c57, 0x2c57, 0x2c57,
    0x2c57, 0x2c57, 0x2c57, 0x2c57, 0x2c57, 0x2c57, 0x2c57, 0x2c57,
};

static const uint8_t dec_23[] = {
      11,   12,   15,   30,   63,   63,   63,   63,
      12,   13,   17,   42,   63,   63,   63,   63,
      15,   17,   36,   63,   63,   63,   63,   63,
      30,   42,   63,   63,   63,   63,   63,   63,
      63,   63,   63,   63,   63,   63,   63,   63,
      63,   63,   63,   63,   63,   63,   63,   63,
      63,   63,   63,   63,   63,   63,   63,   63,
      63,   63,   63,   63,   63,   63,   63,   63,
};

static const uint16_t enc_23[] = {
    0x1dd1, 0x1d55, 0x1d55, 0x1c44, 0x1cec, 0x1c44, 0x2444, 0x2788,
    0x2788, 0x2444, 0x2c10, 0x2e18, 0x2f1c, 0x2e18, 0x2c10, 0x2c10,
    0x2c10, 0x2c10, 0x2c10, 0x2c10, 0x2c10, 0x2c10, 0x2c10, 0x2c10,
    0x2c10, 0x2c10, 0x2c10, 0x2c10, 0x2c10, 0x2c10, 0x2c10, 0x2c10,
    0x2c10, 0x2c10, 0x2c10, 0x2c10, 0x2c10, 0x2c10, 0x2c10, 0x2c10,
    0x2c10, 0x2c10, 0x2c10, 0x2c10, 0x2c10, 0x2c10, 0x2c10, 0x2c10,
    0x2c10, 0x2c10, 0x2c10, 0x2c10, 0x2c10, 0x2c10, 0x2c10, 0x2c10,
    0x2c10, 0x2c10, 0x2c10, 0x2c10, 0x2c10, 0x2c10, 0x2c10, 0x2c10,
};

static const uint8_t dec_24[] = {
      12,   12,   16,   32,   67,   67,   67,   67,
      12,   14,   18,   45,   67,   67,   67,   67,
      16,   18,   38,   67,   67,   67,   67,   67,
      32,   45,   67,   67,   67,   67,   67,   67,
      67,   67,   67,   67,   67,   67,   67,   67,
      67,   67,   67,   67,   67,   67,   67,   67,
      67,   67,   67,   67,   67,   67,   67,   67,
      67,   67,   67,   67,   67,   67,   67,   67,
};

static const uint16_t enc_24[] = {
    0x1d55, 0x1d55, 0x1d55, 0x1c00, 0x1c92, 0x1c00, 0x2400, 0x271c,
    0x271c, 0x2400, 0x37a4, 0x2db0, 0x2ebd, 0x2db0, 0x37a4, 0x37a4,
    0x37a4, 0x37a4, 0x37a4, 0x37a4, 0x37a4, 0x37a4, 0x37a4, 0x37a4,
    0x37a4, 0x37a4, 0x37a4, 0x37a4, 0x37a4, 0x37a4, 0x37a4, 0x37a4,
    0x37a4, 0x37a4, 0x37a4, 0x37a4, 0x37a4, 0x37a4, 0x37a4, 0x37a4,
    0x37a4, 0x37a4, 0x37a4, 0x37a4, 0x37a4, 0x37a4, 0x37a4, 0x37a4,
    0x37a4, 0x37a4, 0x37a4, 0x37a4, 0x37a4, 0x37a4, 0x37a4, 0x37a4,
    0x37a4, 0x37a4, 0x37a4, 0x37a4, 0x37a4, 0x37a4, 0x37a4, 0x37a4,
};

static const uint8_t dec_25[] = {
      12,   13,   17,   34,   71,   71,   71,   71,
      13,   15,   19,   48,   71,   71,   71,   71,
      17,   19,   40,   71,   71,   71,   71,   71,
      34,   48,   71,   71,   71,   71,   71,   71,
      71,   71,   71,   71,   71,   71,   71,   71,
      71,   71,   71,   71,   71,   71,   71,   71,
      71,   71,   71,   71,   71,   71,   71,   71,
      71,   71,   71,   71,   71,   71,   71,   71,
};

static const uint16_t enc_25[] = {
    0x1d55, 0x1cec, 0x1cec, 0x2788, 0x1c44, 0x2788, 0x2f88, 0x26bd,
    0x26bd, 0x2f88, 0x3736, 0x2d55, 0x2e66, 0x2d55, 0x3736, 0x3736,
    0x3736, 0x3736, 0x3736, 0x3736, 0x3736, 0x3736, 0x3736, 0x3736,
    0x3736, 0x3736, 0x3736, 0x3736, 0x3736, 0x3736, 0x3736, 0x3736,
    0x3736, 0x3736, 0x3736, 0x3736, 0x3736, 0x3736, 0x3736, 0x3736,
    0x3736, 0x3736, 0x3736, 0x3736, 0x3736, 0x3736, 0x3736, 0x3736,
    0x3736, 0x3736, 0x3736, 0x3736, 0x3736, 0x3736, 0x3736, 0x3736,
    0x3736, 0x3736, 0x3736, 0x3736, 0x3736, 0x3736, 0x3736, 0x3736,
};

static const uint8_t dec_26[] = {
      13,   14,   18,   36,   75,   75,   75,   75,
      14,   16,   20,   50,   75,   75,   75,   75,
      18,   20,   43,   75,   75,   75,   75,   75,
      36,   50,   75,   75,   75,   75,   75,   75,
      75,   75,   75,   75,   75,   75,   75,   75,
      75,   75,   75,   75,   75,   75,   75,   75,
      75,   75,   75,   75,   75,   75,   75,   75,
      75,   75,   75,   75,   75,   75,   75,   75,
};

static const uint16_t enc_26[] = {
    0x1cec, 0x1c92, 0x1c92, 0x271c, 0x1c00, 0x271c, 0x2f1c, 0x2666,
    0x2666, 0x2f1c, 0x36d4, 0x2d1f, 0x2df4, 0x2d1f, 0x36d4, 0x36d4,
    0x36d4, 0x36d4, 0x36d4, 0x36d4, 0x36d4, 0x36d4, 0x36d4, 0x36d4,
    0x36d4, 0x36d4, 0x36d4, 0x36d4, 0x36d4, 0x36d4, 0x36d4, 0x36d4,
    0x36d4, 0x36d4, 0x36d4, 0x36d4, 0x36d4, 0x36d4, 0x36d4, 0x36d4,
    0x36d4, 0x36d4, 0x36d4, 0x36d4, 0x36d4, 0x36d4, 0x36d4, 0x36d4,
    0x36d4, 0x36d4, 0x36d4, 0x36d4, 0x36d4, 0x36d4, 0x36d4, 0x36d4,
    0x36d4, 0x36d4, 0x36d4, 0x36d4, 0x36d4, 0x36d4, 0x36d4, 0x36d4,
};

static const uint8_t dec_27[] = {
      14,   14,   19,   38,   79,   79,   79,   79,
      14,   17,   21,   53,   79,   79,   79,   79,
      19,   21,   45,   79,   79,   79,   79,   79,
      38,   53,   79,   79,   79,   79,   79,   79,
      79,   79,   79,   79,   79,   79,   79,   79,
      79,   79,   79,   79,   79,   79,   79,   79,
      79,   79,   79,   79,   79,   79,   79,   79,
      79,   79,   79,   79,   79,   79,   79,   79,
};

static const uint16_t enc_27[] = {
    0x1c92, 0x1c92, 0x1c92, 0x26bd, 0x2788, 0x26bd, 0x2ebd, 0x2618,
    0x2618, 0x2ebd, 0x367b, 0x2cd5, 0x2db0, 0x2cd5, 0x367b, 0x367b,
    0x367b, 0x367b, 0x367b, 0x367b, 0x367b, 0x367b, 0x367b, 0x367b,
    0x367b, 0x367b, 0x367b, 0x367b, 0x367b, 0x367b, 0x367b, 0x367b,
    0x367b, 0x367b, 0x367b, 0x367b, 0x367b, 0x367b, 0x367b, 0x367b,
    0x367b, 0x367b, 0x367b, 0x367b, 0x367b, 0x367b, 0x367b, 0x367b,
    0x367b, 0x367b, 0x367b, 0x367b, 0x367b, 0x367b, 0x367b, 0x367b,
    0x367b, 0x367b, 0x367b, 0x367b, 0x367b, 0x367b, 0x367b, 0x367b,
};

struct jpeg_quant_table_t {
    int qualtity;
    const uint8_t *dec_table;
    const uint16_t *enc_table;
};

static const struct jpeg_quant_table_t jpeg_quant_tables[] = {
    { -1, dec_0, enc_0 }, // index 0
    { -1, dec_1, enc_1 }, // index 1
    { -1, dec_2, enc_2 }, // index 2
    { -1, dec_3, enc_3 }, // index 3
    { -1, dec_4, enc_4 }, // index 4
    { -1, dec_5, enc_5 }, // index 5
    { -1, dec_6, enc_6 }, // index 6
    { -1, dec_7, enc_7 }, // index 7
    { -1, dec_8, enc_8 }, // index 8
    { -1, dec_9, enc_9 }, // index 9
    { -1, dec_10, enc_10 }, // index 10
    { -1, dec_11, enc_11 }, // index 11
    { -1, dec_12, enc_12 }, // index 12
    { -1, dec_13, enc_13 }, // index 13
    { -1, dec_14, enc_14 }, // index 14
    { -1, dec_15, enc_15 }, // index 15
    { -1, dec_16, enc_16 }, // index 16
    { -1, dec_17, enc_17 }, // index 17
    { -1, dec_18, enc_18 }, // index 18
    { -1, dec_19, enc_19 }, // index 19
    { -1, dec_20, enc_20 }, // index 20
    { -1, dec_21, enc_21 }, // index 21
    { -1, dec_22, enc_22 }, // index 22
    { -1, dec_23, enc_23 }, // index 23
    { -1, dec_24, enc_24 }, // index 24
    { -1, dec_25, enc_25 }, // index 25
    { -1, dec_26, enc_26 }, // index 26
    { -1, dec_27, enc_27 }, // index 27
};

#define JPEG_QUANT_TABLES_SIZE (sizeof(jpeg_quant_tables)/sizeof(jpeg_quant_tables[0]))
