#ifndef SPE_H
#define SPE_H

#include <libspe2.h>

typedef struct {
        unsigned long long ea_in;
            unsigned long long ea_out;
} params_t;

typedef struct {
        spe_context_ptr_t spe;
            params_t *params;
} thread_arg_t;

#endif
