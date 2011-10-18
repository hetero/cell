#include <inttypes.h>
#include <math.h>
#include <stdlib.h>

#include "tables.h"
#include "types.h"

#define ISQRT2 0.70710678118654f

static void transpose_block(float *in_data, float *out_data)
{
    int i,j;
    for (i=0; i<8; ++i)
        for (j=0; j<8; ++j)
        {
            out_data[i*8+j] = in_data[j*8+i];
        }
}

static void vec_transpose_block(VF *in_data, VF *out_data)
{
    transpose_block((float *)in_data, (float *)out_data);
}

static void vec_dct_1d(VF *in_data, VF *out_data)
{
    int j;
    float *f_tmp;
    float *f_out = (float *)out_data;

    VF tmp[2];
    VF vec_zero = (VF) {0, 0, 0, 0};

    for (j=0; j<8; ++j)
    {
        tmp[0] = vec_madd(in_data[0], vec_dctlookup[j][0], vec_zero);
        tmp[1] = vec_madd(in_data[1], vec_dctlookup[j][1], tmp[0]);
        f_tmp = (float *)(&tmp[1]);
        f_out[j] = f_tmp[0] + f_tmp[1] + f_tmp[2] + f_tmp[3];
    }
}

static void dct_1d(float *in_data, float *out_data)
{
    int i,j;

    for (j=0; j<8; ++j)
    {
        float dct = 0;

        for (i=0; i<8; ++i)
        {
            dct += in_data[i] * dctlookup[i][j];
        }

        out_data[j] = dct;
    }
}

static void idct_1d(float *in_data, float *out_data)
{
    int i,j;

    for (j=0; j<8; ++j)
    {
        float idct = 0;

        for (i=0; i<8; ++i)
        {
            idct += in_data[i] * dctlookup[j][i];
        }

        out_data[j] = idct;
    }
}

static void vec_scale_block(VF *in_data, VF *out_data)
{
    VF vec_zero = (VF) {0.f, 0.f, 0.f, 0.f};
    VF first_row = (VF) {ISQRT2, ISQRT2, ISQRT2, ISQRT2};
    VF first_column = (VF) {ISQRT2, 1.f, 1.f, 1.f};
    
    out_data[0] = vec_madd(in_data[0], first_row, vec_zero);
    out_data[0] = vec_madd(out_data[0], first_column, vec_zero);
    out_data[1] = vec_madd(in_data[1], first_row, vec_zero);
    
    int i;
    for (i = 2; i < 8; i += 2)
    {
        out_data[i] = vec_madd(in_data[i], first_column, vec_zero);
    }
    for (i = 3; i < 8; i += 2)
    {
        out_data[i] = in_data[i];
    }
}

static void scale_block(float *in_data, float *out_data)
{
    int u,v;

    for (v=0; v<8; ++v)
    {
        for (u=0; u<8; ++u)
        {
            float a1 = !u ? ISQRT2 : 1.0f;
            float a2 = !v ? ISQRT2 : 1.0f;

            /* Scale according to normalizing function */

            out_data[v*8+u] = in_data[v*8+u] * a1 * a2;
        }
    }
}

static void quantize_block(float *in_data, float *out_data, uint8_t *quant_tbl)
{
    int zigzag;
    for (zigzag=0; zigzag < 64; ++zigzag)
    {
        uint8_t u = zigzag_U[zigzag];
        uint8_t v = zigzag_V[zigzag];

        float dct = in_data[v*8+u];

        /* Zig-zag and quantize */
        out_data[zigzag] = round((dct / 4.0) / quant_tbl[zigzag]);
    }
}

static void dequantize_block(float *in_data, float *out_data, uint8_t *quant_tbl)
{
    int zigzag;
    for (zigzag=0; zigzag < 64; ++zigzag)
    {
        uint8_t u = zigzag_U[zigzag];
        uint8_t v = zigzag_V[zigzag];

        float dct = in_data[zigzag];

        /* Zig-zag and de-quantize */
        out_data[v*8+u] = round((dct * quant_tbl[zigzag]) / 4.0);
    }
}

void dct_quant_block_8x8(VSI *in_data, int16_t *out_data, uint8_t *quant_tbl)
{
    VF mb[8*2] __attribute((aligned(16)));
    VF mb2[8*2] __attribute((aligned(16)));

    int i, v;

    for (i=0; i<16; ++i)
        mb2[i] = vec_ctf(in_data[i], 0);

    for (v=0; v<8; ++v)
    {
        vec_dct_1d(mb2+v*2, mb+v*2);
    }

    vec_transpose_block(mb, mb2);

    for (v=0; v<8; ++v)
    {
        vec_dct_1d(mb2+v*2, mb+v*2);
    }

    vec_transpose_block(mb, mb2);
    vec_scale_block(mb2, mb);
    quantize_block((float *)mb, (float *)mb2, quant_tbl);

    for (i=0; i<64; ++i)
        out_data[i] = ((float *)mb2)[i];
}


void dequant_idct_block_8x8(int16_t *in_data, int16_t *out_data, uint8_t *quant_tbl)
{
    float mb[8*8] __attribute((aligned(16)));
    float mb2[8*8] __attribute((aligned(16)));

    int i, v;

    for (i=0; i<64; ++i)
        mb[i] = in_data[i];

    dequantize_block(mb, mb2, quant_tbl);

    scale_block(mb2, mb);

    for (v=0; v<8; ++v)
    {
        idct_1d(mb+v*8, mb2+v*8);
    }

    transpose_block(mb2, mb);

    for (v=0; v<8; ++v)
    {
        idct_1d(mb+v*8, mb2+v*8);
    }

    transpose_block(mb2, mb);

    for (i=0; i<64; ++i)
        out_data[i] = mb[i];
}

void sad_block_8x8(uint8_t *block1, uint8_t *block2, int stride, int *result)
{
    *result = 0;

    int u,v;
    for (v=0; v<8; ++v)
        for (u=0; u<8; ++u)
            *result += abs(block2[v*stride+u] - block1[v*stride+u]);
}
