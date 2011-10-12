#ifndef SPE_H
#define SPE_H

typedef struct {
            int sad, x, y, pad;
} sad_out_t;

typedef struct {
    unsigned long long orig;
    unsigned long long ref;
    unsigned long long out;
    int w;
    int shift;
} sad_params_t;

#endif
