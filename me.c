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

    return NULL;
}

void me_cell(uint8_t *orig, uint8_t *ref, sad_out_t *sad_out, int w, int shift) {
    const int NUM_SPE = 8;
    int i, y, ret;
    sad_out_t spe_out[NUM_SPE] __attribute__((aligned(128)));
    sad_params_t sad_params[NUM_SPE] __attribute__((aligned(128)));;

    spe_program_handle_t *prog;
    spe_context_ptr_t spe[NUM_SPE];
    pthread_t thread[NUM_SPE];
    thread_arg_t arg[NUM_SPE];

    prog = spe_image_open("sad_spe.elf");
    if (!prog) {
        perror("spe_image_open");
        exit(1);
    }

    for (i = 0; i < NUM_SPE; i++) {
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

    for (i = 0, y = 0; i < NUM_SPE; i++, y += 4) {
        sad_params[i].orig = (unsigned long) &orig[0];
        sad_params[i].ref = (unsigned long) &ref[y*w];
        sad_params[i].out = (unsigned long) &spe_out[i];
        sad_params[i].w = w;
        sad_params[i].shift = shift;

        arg[i].spe = spe[i];
        arg[i].params = &sad_params[i];

        ret = pthread_create(&thread[i], NULL, run_sad_spe, &arg[i]);
        if (ret) {
            perror("pthread_create");
            exit(1);
        }
    }
    
    for (i = 0; i < NUM_SPE; i++) {
        pthread_join(thread[i], NULL);
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
    
    for (i = 0, y = 0; i < NUM_SPE; i++, y += 4) {
        if (spe_out[i].sad < sad_out->sad)
        {
            sad_out->x = spe_out[i].x;
            sad_out->y = y + spe_out[i].y;
            sad_out->sad = spe_out[i].sad;
        }
    }
}

/* Motion estimation for 8x8 block */
static void me_block_8x8(struct c63_common *cm, int mb_x, int mb_y, uint8_t *orig, uint8_t *ref, int cc)
{
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


    int x,y;
    int mx = mb_x * 8;
    int my = mb_y * 8;

    int best_sad = INT_MAX;

    if (bottom - top != 32) // TODO 32 constant
    {
        // border-case (can be done better)
        for (y=top; y<bottom; ++y)
        {
            for (x=left; x<right; ++x)
            {
                int sad;
                sad_block_8x8(orig + my*w+mx, ref + y*w+x, w, &sad);

    //            printf("(%4d,%4d) - %d\n", x, y, sad);

                if (sad < best_sad)
                {
                    mb->mv_x = x - mx;
                    mb->mv_y = y - my;
                    best_sad = sad;
                }
            }
        }
    }
    else
    {
        // main case (bottom-top == 32)
        int orig_offset = 0, ref_offset = 0;
        sad_out_t sad_out;
        sad_out.sad = INT_MAX;
        sad_out.x = 0;
        sad_out.y = 0;
        if (mb_x % 2 == 1)
            orig_offset = -8;
        if (left % 16 == 8)
            ref_offset = -8;
            me_cell(orig + my*w + mx + orig_offset, 
                    ref + top*w + left + ref_offset, &sad_out, w, 0); 
        
        if (sad_out.sad < best_sad)
        {
            mb->mv_x = left + sad_out.x - mx;
            mb->mv_y = top + sad_out.y - my;
            best_sad = sad_out.sad;
        }
    }

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
}

void c63_motion_estimate(struct c63_common *cm)
{
    /* Compare this frame with previous reconstructed frame */

    int mb_x, mb_y;

    /* Luma */
    for (mb_y=0; mb_y < cm->mb_rows; ++mb_y)
    {
        for (mb_x=0; mb_x < cm->mb_cols; ++mb_x)
        {
            me_block_8x8(cm, mb_x, mb_y, cm->curframe->orig->Y, cm->refframe->recons->Y, 0);
        }
    }

    /* Chroma */
    for (mb_y=0; mb_y < cm->mb_rows/2; ++mb_y)
    {
        for (mb_x=0; mb_x < cm->mb_cols/2; ++mb_x)
        {
            me_block_8x8(cm, mb_x, mb_y, cm->curframe->orig->U, cm->refframe->recons->U, 1);
            me_block_8x8(cm, mb_x, mb_y, cm->curframe->orig->V, cm->refframe->recons->V, 2);
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

