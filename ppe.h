#ifndef PPE_H
#define PPE_H

#include <libspe2.h>
#include "spe.h"

typedef struct {
    spe_context_ptr_t spe;
    sad_params_t *params;
} thread_arg_t;

#endif
