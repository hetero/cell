#ifndef PPE_H
#define PPE_H

#include <libspe2.h>
#include "spe.h"

spe_context_ptr_t spe[8] __attribute__((aligned(128)));

typedef struct {
    spe_context_ptr_t spe;
    sad_params_t *params;
} thread_arg_t;

#endif
