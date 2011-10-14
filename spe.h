#ifndef SPE_H
#define SPE_H

typedef struct {
            int sad, x, y, pad;
} sad_out_t;

typedef struct {
    unsigned long long orig;
    unsigned long long ref;
    unsigned long long sad_out;
    unsigned long long mb;
    int w;
    int wm128;
    int orig_offset;
    int ref_offset;
    int orig_x;
    int orig_y;
    int ref_w;
    int ref_h;
} sad_params_t;

#endif
