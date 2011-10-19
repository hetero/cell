#ifndef CPU_MJPEG_H
#define CPU_MJPEG_H

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <inttypes.h>
#include <pthread.h>
#include "ppe.h"

#define MAX_FILELENGTH 200
#define DEFAULT_OUTPUT_FILE "a.mjpg"

#define ISQRT2 0.70710678118654f
#define PI 3.14159265358979
#define ILOG2 1.442695040888963 // 1/log(2);

#define COLOR_COMPONENTS 3

#define YX 2
#define YY 2
#define UX 1
#define UY 1
#define VX 1
#define VY 1

#define MIN(a,b) ((a) < (b) ? (a) : (b))
#define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))

#define WAIT_MODE 1
#define SAD_MODE 2
#define OFF_MODE 3
#define DCT_MODE 4
#define IDCT_MODE 5

extern spe_context_ptr_t spe[8];
extern int mode;
extern pthread_mutex_t mutex;

extern pthread_t smart_thread[NUM_SPE];

extern int SPE_NUMBERS[6];

extern int global_mb_x, global_mb_y, global_mb_rows, global_mb_cols, global_cc;
extern uint8_t *global_orig; 
extern uint8_t *global_ref;
extern struct c63_common *global_cm;


extern int g_dct_col, g_dct_row, g_dct_width, g_dct_height, g_dct_quantization;
extern uint8_t *g_dct_in_data, *g_dct_prediction;
extern int16_t *g_dct_out_data;

extern int g_idct_row, g_idct_col, g_idct_width, g_idct_height, g_idct_quantization;
extern int16_t *g_idct_in_data;
extern uint8_t *g_idct_prediction, *g_idct_out_data;


void lock();
void unlock();
void *run_smart_thread(void *void_spe_nr);

struct yuv
{
  uint8_t *Y;
  uint8_t *U;
  uint8_t *V;

  uint8_t *Ybase;
  uint8_t *Ubase;
  uint8_t *Vbase;
};

struct dct
{
  int16_t *Ydct;
  int16_t *Udct;
  int16_t *Vdct;
};

typedef struct yuv yuv_t;
typedef struct dct dct_t;

struct entropy_ctx
{
    FILE *fp;
    unsigned int bit_buffer;
    unsigned int bit_buffer_width;
};

struct macroblock
{
    int use_mv;
    int8_t mv_x, mv_y;
};

struct frame
{
    yuv_t *orig;        // Original input image
    yuv_t *recons;      // Reconstructed image
    yuv_t *predicted;   // Predicted frame from intra-prediction

    dct_t *residuals;   // Difference between original image and predicted frame

    struct macroblock *mbs[3];

    int keyframe;
};

struct c63_common
{
    int width, height;
    int ypw, yph, upw, uph, vpw, vph;

    int padw[3], padh[3];

    int mb_cols, mb_rows;

    uint8_t qp;                         // Quality parameter

    int me_search_range;


    uint8_t quanttbl[3][64];

    struct frame *refframe;
    struct frame *curframe;

    int framenum;

    int keyframe_interval;
    int frames_since_keyframe;

    struct entropy_ctx e_ctx;
};

void put_bytes(FILE *fp, const void* data, unsigned int len);
void put_byte(FILE *fp, int byte);
void put_bits(struct entropy_ctx *c, uint16_t bits, uint8_t n);
void flush_bits(struct entropy_ctx *c);
uint8_t get_byte(FILE *fp);
int read_bytes(FILE *fp, void *data, unsigned int sz);
uint16_t get_bits(struct entropy_ctx *c, uint8_t n);

void dct_quant_block_8x8(int16_t *in_data, int16_t *out_data, uint8_t *quant_tbl);
void dequant_idct_block_8x8(int16_t *in_data, int16_t *out_data, uint8_t *quant_tbl);

void write_frame(struct c63_common *cm);
void dequantize_idct(int16_t *in_data, uint8_t *prediction, uint32_t width, uint32_t height,
			 uint8_t *out_data, uint8_t *quantization);
void dequantize_idct_spu(int16_t *in_data, uint8_t *prediction, uint32_t width, uint32_t height,
			 uint8_t *out_data, int quantization);
void dct_quantize(uint8_t *in_data, uint8_t *prediction,
        uint32_t width, uint32_t height,
        int16_t *out_data, uint8_t *quant_tbl, int quantization);

void destroy_frame(struct frame *f);
struct frame* create_frame(struct c63_common *cm, yuv_t *image);
void c63_motion_estimate(struct c63_common *cm);
void c63_motion_compensate(struct c63_common *cm);

void dump_image(yuv_t *image, int w, int h, FILE *fp);

#endif /* mjpeg_encoder.h */
