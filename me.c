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

#include "c63.h"

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

void me_cell(uint8_t *orig, uint8_t *ref, sad_out_t *sad_out, int w) {
    const int NUM_SPE = 8;
    int spe, y, ret;
    sad_out_t spe_out[8];
/*    sad_params_t sad_params[NUM_SPE] __attribute__((aligned(16)));;

    spe_program_handle_t *prog;
    spe_context_ptr_t spe[NUM_SPE];
    pthread_t thread[NUM_SPE];
    thread_arg_t arg[NUM_SPE];

    prog = spe_image_open("sad_spe.elf");
*/
    for (y = 0, spe = 0; spe < 8; y += 4, spe++) {
        sad_4rows(orig, ref + y*w, &spe_out[spe], w);
    }
    
    for (y = 0, spe = 0; spe < 8; y += 4, spe++) {
        if (spe_out[spe].sad < sad_out->sad)
        {
            sad_out->x = spe_out[spe].x;
            sad_out->y = y + spe_out[spe].y;
            sad_out->sad = spe_out[spe].sad;
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
        sad_out_t sad_out;
        sad_out.sad = INT_MAX;
        sad_out.x = 0;
        sad_out.y = 0;
        me_cell(orig + my*w + mx, ref + top*w + left, &sad_out, w); 
        
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

