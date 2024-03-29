#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <getopt.h>
#include <errno.h>
#include <stdint.h>
#include <math.h>
#include <assert.h>

#include "c63.h"
#include "tables.h"
#include "spe.h"

void dequantize_idct_row(int16_t *in_data, uint8_t *prediction, int w, int h, int y,
			 uint8_t *out_data, uint8_t *quantization)
{
    int x;

    int16_t block[8*8];

    /* Perform the dequantization and iDCT */
    for(x = 0; x < w; x += 8)
    {
        int i,j;
        dequant_idct_block_8x8(in_data+(x*8), block, quantization);


        for (i=0; i<8; ++i)
            for (j=0; j<8; ++j)
            {
                /* Add prediction block. Note: DCT is not precise - Clamp to legal values */
                int16_t tmp = block[i*8+j] + (int16_t)prediction[i*w+j+x];
                if (tmp < 0)
                    tmp = 0;
                else if (tmp > 255)
                    tmp = 255;

                out_data[i*w+j+x] = tmp;
            }
    }
}

void dequantize_idct(int16_t *in_data, uint8_t *prediction, uint32_t width, uint32_t height,
			 uint8_t *out_data, uint8_t *quantization)
{
    
    int y;
    for (y=0; y<height; y+=8)
    {
        dequantize_idct_row(in_data+y*width, prediction+y*width, width, height, y, out_data+y*width, quantization);
    }
}

void dequantize_idct_spu(int16_t *in_data, uint8_t *prediction, uint32_t width, uint32_t height,
			 uint8_t *out_data, int quantization)
{    
    lock();
    mode = IDCT_MODE;
    g_idct_row = 0;
    g_idct_col = 0;
    g_idct_width = width;
    g_idct_height = height;
    g_idct_in_data = in_data;
    g_idct_prediction = prediction;
    g_idct_quantization = quantization;
    g_idct_out_data = out_data;
    unlock();
    int i;
    int spe_nr = rand() % NUM_SPE;
    for (i = 0; i < NUM_SPE; i++) {
        int ret = pthread_create(&smart_thread[spe_nr], NULL, run_smart_thread, 
               &SPE_NUMBERS[spe_nr]); 
        if (ret) {
            perror("pthread_create");
            exit(1);
        }
        spe_nr = (spe_nr + 1) % NUM_SPE;
    }

    for (spe_nr = 0; spe_nr < NUM_SPE; spe_nr++) 
        pthread_join(smart_thread[spe_nr], NULL);
}

void print_common(int16_t *data)
{
    int i, j;
    printf("First block before DCT:\n");
    for (i = 0; i < 8; i++)
    {
        for (j = 0; j < 8; j++)
        {
            printf("%d ", data[i * 8 + j]);
        }
        printf("\n");
    }
}


void dct_quantize_row(uint8_t *in_data, uint8_t *prediction, int w, int h,
        int16_t *out_data, uint8_t *quantization)
{
    int x;

    int16_t block[8*8];

    /* Perform the DCT and quantization */
    for(x = 0; x < w; x += 8)
    {
        int i,j;
        for (i=0; i<8; ++i)
            for (j=0; j<8; ++j)
                block[i*8+j] = ((int16_t)in_data[i*w+j+x] - prediction[i*w+j+x]);

        /* Store MBs linear in memory, i.e. the 64 coefficients are stored continous.
         * This allows us to ignore stride in DCT/iDCT and other functions. */
        dct_quant_block_8x8(block, out_data+(x*8), quantization);
        // DEBUG
        static int first = 1;
        if (first)
        {
//            print_block_ble(out_data);
            print_common(block);
            first = 0;
        }
        
        // DEBUG
    }
}

void dct_quantize(uint8_t *in_data, uint8_t *prediction,
        uint32_t width, uint32_t height,
        int16_t *out_data, uint8_t *quant_tbl, int quantization)
{
/*
    int y;
    for (y=0; y<height; y+=8)
    {
        dct_quantize_row(in_data+y*width, prediction+y*width, width, height, out_data+y*width, quant_tbl);
    }
  */
    
    lock();
    mode = DCT_MODE;
    g_dct_row = 0;
    g_dct_col = 0;
    g_dct_width = width;
    g_dct_height = height;
    g_dct_in_data = in_data;
    g_dct_prediction = prediction;
    g_dct_quantization = quantization;
    g_dct_out_data = out_data;
    unlock();
    int i;
    int spe_nr = rand() % NUM_SPE;
    for (i = 0; i < NUM_SPE; i++) {
        int ret = pthread_create(&smart_thread[spe_nr], NULL, run_smart_thread, 
               &SPE_NUMBERS[spe_nr]); 
        if (ret) {
            perror("pthread_create");
            exit(1);
        }
        spe_nr = (spe_nr + 1) % NUM_SPE;
    }

    for (spe_nr = 0; spe_nr < NUM_SPE; spe_nr++) 
        pthread_join(smart_thread[spe_nr], NULL);
        
}

void destroy_frame(struct frame *f)
{
    if (!f) // First frame
        return;

    free(f->recons->Y);
    free(f->recons->U);
    free(f->recons->V);
    free(f->recons);

    free(f->residuals->Ydct);
    free(f->residuals->Udct);
    free(f->residuals->Vdct);
    free(f->residuals);

    free(f->predicted->Y);
    free(f->predicted->U);
    free(f->predicted->V);
    free(f->predicted);

    free(f->mbs[0]);
    free(f->mbs[1]);
    free(f->mbs[2]);

    free(f);
}

struct frame* create_frame(struct c63_common *cm, yuv_t *image)
{
    int nothing;

    struct frame *f = NULL;
    nothing = posix_memalign((void **) &f, 128, sizeof(struct frame));

    f->orig = image;

    nothing = posix_memalign((void **) &(f->recons), 128, sizeof(yuv_t));
    nothing = posix_memalign((void **) &(f->recons->Y), 128, cm->ypw * cm->yph);
    nothing = posix_memalign((void **) &(f->recons->U), 128, cm->upw * cm->uph);
    nothing = posix_memalign((void **) &(f->recons->V), 128, cm->vpw * cm->vph);

    nothing = posix_memalign((void **) &(f->predicted), 128, sizeof(yuv_t));
    nothing = posix_memalign((void **) &(f->predicted->Y), 128, cm->ypw * cm->yph);
    nothing = posix_memalign((void **) &(f->predicted->U), 128, cm->upw * cm->uph);
    nothing = posix_memalign((void **) &(f->predicted->V), 128, cm->vpw * cm->vph);

    memset(f->predicted->Y, 0x80, cm->ypw * cm->yph);
    memset(f->predicted->U, 0x80, cm->upw * cm->uph);
    memset(f->predicted->V, 0x80, cm->vpw * cm->vph);

    nothing = posix_memalign((void **) &(f->residuals), 128, sizeof(dct_t));
    nothing = posix_memalign((void **) &(f->residuals->Ydct), 128, 
        cm->ypw * cm->yph * sizeof(int16_t));
    nothing = posix_memalign((void **) &(f->residuals->Udct), 128, 
        cm->upw * cm->uph * sizeof(int16_t));
    nothing = posix_memalign((void **) &(f->residuals->Vdct), 128, 
        cm->vpw * cm->vph * sizeof(int16_t));

    memset(f->residuals->Ydct, 0x80, cm->ypw * cm->yph * sizeof(int16_t));
    memset(f->residuals->Udct, 0x80, cm->upw * cm->uph * sizeof(int16_t));
    memset(f->residuals->Vdct, 0x80, cm->vpw * cm->vph * sizeof(int16_t));
    
    nothing = posix_memalign((void **) &(f->mbs[0]), 128,
       cm->ypw * cm->yph * sizeof(struct macroblock));
    nothing = posix_memalign((void **) &(f->mbs[1]), 128,
       cm->upw * cm->uph * sizeof(struct macroblock));
    nothing = posix_memalign((void **) &(f->mbs[2]), 128,
       cm->vpw * cm->vph * sizeof(struct macroblock));

    memset(f->mbs[0], 0, cm->ypw * cm->yph * sizeof(struct macroblock));
    memset(f->mbs[1], 0, cm->upw * cm->uph * sizeof(struct macroblock));
    memset(f->mbs[2], 0, cm->vpw * cm->vph * sizeof(struct macroblock));

    return f;
}

void dump_image(yuv_t *image, int w, int h, FILE *fp)
{
    fwrite(image->Y, 1, w*h, fp);
    fwrite(image->U, 1, w*h/4, fp);
    fwrite(image->V, 1, w*h/4, fp);
}

