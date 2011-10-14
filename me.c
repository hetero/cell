#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <getopt.h>
#include <errno.h>
#include <stdint.h>
#include <math.h>
#include <assert.h>
#include <limits.h>
#include <libspe2.h>
#include <pthread.h>

#include "c63.h"
#include "ppe.h"

#define NUM_SPE 8

int started[NUM_SPE];
sad_out_t spe_out[NUM_SPE] __attribute__((aligned(128)));
sad_params_t sad_params[NUM_SPE] __attribute__((aligned(128)));

spe_context_ptr_t spe[NUM_SPE];
pthread_t thread[NUM_SPE];
thread_arg_t arg[NUM_SPE];

spe_program_handle_t *prog;

void spe_init() {
    int i, ret;
    prog = spe_image_open("sad_spe.elf");
    if (!prog) {
        perror("spe_image_open");
        exit(1);
    }

    for (i = 0; i < NUM_SPE; i++) {
        started[i] = 0;
        spe[i] = spe_context_create(0, NULL);
        if (!spe[i]) {
            perror("spe_context_create");
            exit(1);
        }

        ret = spe_program_load(spe[i], prog);
        if (ret) {
            perror("spe_program_load");
            exit(1);
        }
    }
}

void spe_dispose() {    
    int i, ret;
    for (i = 0; i < NUM_SPE; i++) {
        pthread_join(thread[i], NULL);
        started[i] = 0;
        ret = spe_context_destroy(spe[i]);
        if (ret) {
            perror("spe_context_destroy");
            exit(1);
        }
    }

    ret = spe_image_close(prog);
    if (ret) {
        perror("spe_image_close");
        exit(1);
    }
}

void *run_sad_spe(void *thread_arg) {
    int ret;
    thread_arg_t *arg = (thread_arg_t *) thread_arg;
    unsigned int entry;
    spe_stop_info_t stop_info;

    entry = SPE_DEFAULT_ENTRY;
    ret = spe_context_run(arg->spe, &entry, 0, arg->params, NULL, &stop_info);
    if (ret < 0) {
        perror("spe_context_run");
        return NULL;
    }

    sad_out_t *sad_out = (sad_out_t *) (unsigned long) (arg->params->sad_out);
    struct macroblock *mb = (struct macroblock *) (unsigned long) (arg->params->mb);

    mb->mv_x = sad_out->x - arg->params->orig_x;
    mb->mv_y = sad_out->y - arg->params->orig_y;
    int best_sad = sad_out->sad;

    /* We only use motion vectors if the difference is small. */
    if (best_sad < 512)
    {
        mb->use_mv = 1;
//        printf("Using motion vector (%d, %d) with SAD %d\n", mb->mv_x, mb->mv_y, best_sad);
    }
    else
    {
        mb->use_mv = 0;
    }

    return NULL;
}
    

/* Motion estimation for 8x8 block */
static void me_block_8x8(int spe_nr, struct c63_common *cm, int mb_x, int mb_y, uint8_t *orig, uint8_t *ref, int cc)
{
    if (started[spe_nr])
        pthread_join(thread[spe_nr], NULL);
    started[spe_nr] = 1;

    struct macroblock *mb = &cm->curframe->mbs[cc][mb_y * cm->padw[cc]/8 + mb_x];

    int range = cm->me_search_range;

    int left = mb_x*8 - range;
    int top = mb_y*8 - range;
    int right = mb_x*8 + range;
    int bottom = mb_y*8 + range;

    int w = cm->padw[cc];
    int h = cm->padh[cc];

    /* Make sure we are within bounds of reference frame */
    // TODO: Support partial frame bounds
    if (left < 0)
        left = 0;
    if (top < 0)
        top = 0;
    if (right > (w - 8))
        right = w - 8;
    if (bottom > (h - 8))
        bottom = h - 8;

    int mx = mb_x * 8;
    int my = mb_y * 8;
    
    // pointer to orig block
    uint8_t *orig_ptr = orig + my*w + mx;
    sad_params[spe_nr].orig_offset = (unsigned long) orig_ptr % 128;
    orig_ptr -= sad_params[spe_nr].orig_offset;
    sad_params[spe_nr].orig_x = mx - left;
    sad_params[spe_nr].orig_y = my - top;

    // pointer to ref window
    uint8_t *ref_ptr = ref + top*w + left;
    sad_params[spe_nr].ref_offset = (unsigned long) ref_ptr % 128;
    ref_ptr -= sad_params[spe_nr].ref_offset;
    sad_params[spe_nr].ref_w = right - left + 7;
    sad_params[spe_nr].ref_h = bottom - top + 7;

    sad_params[spe_nr].orig = (unsigned long) orig_ptr;
    sad_params[spe_nr].ref = (unsigned long) ref_ptr;
    sad_params[spe_nr].sad_out = (unsigned long) &spe_out[spe_nr];
    sad_params[spe_nr].mb = (unsigned long) mb;
    sad_params[spe_nr].w = w;
    sad_params[spe_nr].wm128 = w % 128;

    arg[spe_nr].spe = spe[spe_nr];
    arg[spe_nr].params = &sad_params[spe_nr];
    
    int ret = pthread_create(&thread[spe_nr], NULL, run_sad_spe, &arg[spe_nr]);
    if (ret) {
        perror("pthread_create");
        exit(1);
    }
}

void c63_motion_estimate(struct c63_common *cm)
{
    /* Compare this frame with previous reconstructed frame */

    int mb_x, mb_y, spe_nr = 0;

    /* Luma */
    for (mb_y=0; mb_y < cm->mb_rows; ++mb_y)
    {
        for (mb_x=0; mb_x < cm->mb_cols; ++mb_x)
        {
            me_block_8x8(spe_nr, cm, mb_x, mb_y, cm->curframe->orig->Y, cm->refframe->recons->Y, 0);
            spe_nr = (spe_nr + 1) % NUM_SPE;
        }
    }

    /* Chroma */
    for (mb_y=0; mb_y < cm->mb_rows/2; ++mb_y)
    {
        for (mb_x=0; mb_x < cm->mb_cols/2; ++mb_x)
        {
            me_block_8x8(spe_nr, cm, mb_x, mb_y, cm->curframe->orig->U, cm->refframe->recons->U, 1);
            spe_nr = (spe_nr + 1) % NUM_SPE;
            me_block_8x8(spe_nr, cm, mb_x, mb_y, cm->curframe->orig->V, cm->refframe->recons->V, 2);
            spe_nr = (spe_nr + 1) % NUM_SPE;
        }
    }
}

/* Motion compensation for 8x8 block */
static void mc_block_8x8(struct c63_common *cm, int mb_x, int mb_y, uint8_t *predicted, uint8_t *ref, int cc)
{
    struct macroblock *mb = &cm->curframe->mbs[cc][mb_y * cm->padw[cc]/8 + mb_x];

    if (!mb->use_mv)
        return;

    int left = mb_x*8;
    int top = mb_y*8;
    int right = left + 8;
    int bottom = top + 8;

    int w = cm->padw[cc];

    /* Copy block from ref mandated by MV */
    int x,y;
    for (y=top; y < bottom; ++y)
    {
        for (x=left; x < right; ++x)
        {
            predicted[y*w+x] = ref[(y + mb->mv_y) * w + (x + mb->mv_x)];
        }
    }
}

void c63_motion_compensate(struct c63_common *cm)
{
    int mb_x, mb_y;

    /* Luma */
    for (mb_y=0; mb_y < cm->mb_rows; ++mb_y)
    {
        for (mb_x=0; mb_x < cm->mb_cols; ++mb_x)
        {
            mc_block_8x8(cm, mb_x, mb_y, cm->curframe->predicted->Y, cm->refframe->recons->Y, 0);
        }
    }

    /* Chroma */
    for (mb_y=0; mb_y < cm->mb_rows/2; ++mb_y)
    {
        for (mb_x=0; mb_x < cm->mb_cols/2; ++mb_x)
        {
            mc_block_8x8(cm, mb_x, mb_y, cm->curframe->predicted->U, cm->refframe->recons->U, 1);
            mc_block_8x8(cm, mb_x, mb_y, cm->curframe->predicted->V, cm->refframe->recons->V, 2);
        }
    }
}

