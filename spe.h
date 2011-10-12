#ifndef SPE_H
#define SPE_H

#include <libspe2.h>

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

typedef struct {
    spe_context_ptr_t spe;
    sad_params_t *params;
} thread_arg_t;

#endif
