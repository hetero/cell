#ifndef SPE_H
#define SPE_H

typedef struct {
            int sad, x, y;
} sad_out_t;

typedef struct {
    unsigned long long orig;
    unsigned long long ref;
    unsigned long long out;
    int w;
    int pad;
} sad_params_t;

#endif
