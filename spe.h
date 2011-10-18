#ifndef SPE_H
#define SPE_H

#define NUM_SPE 6
#define SPE_END 1
#define SPE_FINISH 2

typedef struct {
            int sad, x, y, pad;
} sad_out_t;

typedef struct {
    unsigned long long orig;
    unsigned long long ref;
    unsigned long long sad_out;
    unsigned long long mb;
    int w;
    int spe_nr;
    int orig_offset;
    int ref_offset;
    int orig_x;
    int orig_y;
    int ref_w;
    int ref_h;
} sad_params_t;

typedef unsigned long long ULL;

#endif
