#include "stdint.h"
#include "msb1stimage.hpp"
#include "images.hpp"

int digit32_draw_char(Msb1stImage &dest, int x, int y, char c, PixelOp op) {
    int w, h = 32;
    if ('0' <= c && c <= '9') {
        w = 20;
        dest.draw_image(img_digit32, x, y, w, h, w * (c - '0'), 0, op);
    }
    else if (c == ' ') {
        w = 20;
        // white space
    }
    else if (c == '-') {
        w = 12;
        dest.draw_image(img_digit32, x, y, w, h, 208, 0, op);
    }
    else {
        w = 8;
        dest.draw_image(img_digit32, x, y, w, h, 200, 0, op);
    }
    return w;
}

int digit32_draw_string(Msb1stImage &dest, int x, int y, const char *s, PixelOp op) {
    const char *ptr = s;
    char c;
    while ((c = *(ptr++))) {
        x += digit32_draw_char(dest, x, y, c, op);
        x += 4;
    }
    return x;
}
