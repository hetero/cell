#include <stdio.h>
#include <spu_mfcio.h>
#include <spu_intrinsics.h>
#include "spe.h"
#include "spe_only.h"

#define REF_WIDTH 48
#define VEC_REF_WIDTH REF_WIDTH / 16
#define REF_HEIGHT 39
#define ORIG_WIDTH 8
#define ORIG_HEIGHT 8
#define SAD_WIDTH 32
#define SAD_HEIGHT 32
/*
#define spu_mfcdma64(ls, h, l, sz, tag, cmd) { \
        printf("spu_mfcdma64(%p, %x, %x, %lu, %d, %d) -- Line: %d\n", ls, h, l, sz, tag, cmd, __LINE__); \
    fflush(stdout); \
        spu_mfcdma64(ls, h, l, sz, tag, cmd); \
}
*/
volatile unsigned long long _count;
volatile unsigned int _count_base;
volatile _Bool _counting;

#define DECR_MAX 0xFFFFFFFF
#define DECR_COUNT DECR_MAX

#define prof_clear() \
{ \
    if (_counting) { \
        unsigned int end = spu_readch(SPU_RdDec); \
        _count += (end > _count_base) ? (DECR_MAX + _count_base - end) : \
            (_count_base - end); \
    } \
    _count = 0; \
    _count_base = spu_readch(SPU_RdDec); \
}

#define prof_start() \
{ \
    if (_counting) { \
        unsigned int end = spu_readch(SPU_RdDec); \
        _count += (end > _count_base) ? (DECR_MAX + _count_base - end) : \
            (_count_base - end); \
    } \
    spu_writech(SPU_WrDec, DECR_COUNT); \
    spu_writech(SPU_WrEventMask, MFC_DECREMENTER_EVENT); \
    _counting = 1; \
    _count_base = spu_readch(SPU_RdDec); \
}

#define prof_stop() \
{ \
    if (_counting) { \
        unsigned int end = spu_readch(SPU_RdDec); \
        _count += (end > _count_base) ? (DECR_MAX + _count_base - end) : \
            (_count_base - end); \
    } \
    _counting = 0; \
    spu_writech(SPU_WrEventMask, 0); \
    spu_writech(SPU_WrEventAck, MFC_DECREMENTER_EVENT); \
    _count_base = spu_readch(SPU_RdDec); \
}

#define prof_write() printf("SPU %d: %3.3f\n", spe_nr, (float) _count / 1e6)

float total_time = 0;

uint8_t orig_array[ORIG_WIDTH * ORIG_HEIGHT] __attribute__((aligned(128)));
uint8_t ref_array[REF_WIDTH * REF_HEIGHT] __attribute__((aligned(128)));
uint8_t read_tmp_array[256] __attribute__((aligned(128)));
int sad[SAD_WIDTH * SAD_HEIGHT] __attribute__((aligned(128)));
int ref_rows[REF_HEIGHT];
int vectors;
ULL input_ref;

sad_out_t sad_out __attribute__((aligned(128)));

VUC *orig;
VUC *ref;
VUC *read_tmp;

unsigned mbox_data[4];
int w, spe_nr, orig_x, orig_y, ref_w, ref_h, sad_w, sad_h, ref_offset;

VUC reg;
VUC tmp;
VUC sd;

#define ISQRT2 0.70710678118654f
VSI block[2 * 8];
VF vec_zero = (VF) {0, 0, 0, 0};
VF first_row = (VF) {ISQRT2, ISQRT2, ISQRT2, ISQRT2};
VF first_column = (VF) {ISQRT2, 1.f, 1.f, 1.f};    

void read_row(VUC *dst, ULL src, int size, int offset) {
    int tag = 1;
    int read_size = size + offset;
    read_size += (16 - (read_size%16)) % 16;
    spu_mfcdma64(read_tmp, mfc_ea2h(src), mfc_ea2l(src), 
            (read_size) * sizeof(uint8_t), tag, MFC_GET_CMD);
    spu_writech(MFC_WrTagMask, 1 << tag);
    spu_mfcstat(MFC_TAG_UPDATE_ALL);
}

// read row of input
void read_ref_row(ULL input, int ref_offset, int row_nr) {
    int j;
    input += row_nr * w + ref_offset;
    ref_offset = input % 128;
    input -= ref_offset;

    read_row(read_tmp, input, ref_w, ref_offset);
    // shift vector by vector
    for (j = 0; j < vectors; ++j) {
        ref[row_nr * VEC_REF_WIDTH + j] = spu_shuffle(
                read_tmp[ref_offset / 16 + j],
                read_tmp[ref_offset / 16 + j + 1],
                mask[ref_offset % 16]);
    }

    ref_rows[row_nr] = 1;
}

// stores into "reg" two rows of ref beginnig from x,y
// assumption : REF_WIDTH % 16 = 0
void get_ref(int x, int y) {
    if (!ref_rows[y])
        read_ref_row(input_ref, ref_offset, y);
    if (!ref_rows[y+1])
        read_ref_row(input_ref, ref_offset, y+1);
    int offset = (y*REF_WIDTH + x) % 16;
    int block = y*VEC_REF_WIDTH + x/16;

    reg = spu_shuffle(ref[block], ref[block + 1],  mask[offset]);

    block += VEC_REF_WIDTH;

    tmp = spu_shuffle(ref[block], ref[block + 1], mask[offset]);
    reg = spu_shuffle(reg, tmp, merge_mask);
}

int sad16(VUC orig_reg) {
    sd = spu_absd(orig_reg, reg);
    uint8_t *s = (uint8_t *) &sd;

    return (int) s[0] + (int) s[1] + (int) s[2] + (int) s[3] + (int) s[4] + (int) s[5] + (int) s[6] + (int) s[7]
        + (int) s[8] + (int) s[9] + (int) s[10] + (int) s[11] + (int) s[12] + (int) s[13] + (int) s[14] + (int) s[15];
}

int calc_sad(int x, int y) {
    int i;
    int sum = 0;
    for (i = 0; i < 4; i++) {
        get_ref(x, y);
        sum += sad16(orig[i]);
        y += 2;
    }
    return sum; 
}

int get_sad(int x, int y, const int *vec) {
    x += vec[0];
    y += vec[1];
    if (x < 0 || x >= sad_w || y < 0 || y >= sad_h)
        return INT_MAX;
    if (sad[y*sad_w + x] != -1)
        return sad[y*sad_w + x];
    return calc_sad(x, y);
}

void ds_init() {
    int i;
    for (i = 0; i < sad_w * sad_h; ++i)
        sad[i] = -1;
    for (i = 0; i < ref_h; ++i)
        ref_rows[i] = 0;
    vectors = ref_w / 16 + (ref_w % 16 > 0);
}

void small_ds(int x, int y) {
    int i;
    int best_sad = INT_MAX;
    int best_dir = 0;
    int sad_tmp;
    for (i = 0; i < 5; i++) {
        sad_tmp = get_sad(x, y, &small_diamond[2 * i]);
        if (sad_tmp < best_sad) {
            best_dir = i;
            best_sad = sad_tmp;
        }
    }
    sad_out.x = x + small_diamond[2 * best_dir];
    sad_out.y = y + small_diamond[2 * best_dir + 1];
    sad_out.sad = best_sad;
}

void ds(int x, int y) {
    int i;
    int best_sad = INT_MAX;
    int best_dir = 0;
    int sad_tmp;
    for (i = 0; i < 9; i++) {
        sad_tmp = get_sad(x, y, &big_diamond[2 * i]);
        if (sad_tmp < best_sad) {
            best_dir = i;
            best_sad = sad_tmp;
        }
    }
    if (best_dir == 0) {
        small_ds(x, y);
    }
    else {
        ds(x + big_diamond[2 * best_dir], y + big_diamond[2 * best_dir + 1]);
    }
}

void get_orig_input(ULL input_orig, int orig_offset) {
    int i;
    // (two rows in each loop step)
    for (i = 0; i < ORIG_HEIGHT / 2; ++i) {
        // read 8 bytes and everything on the left to 128 boundary
        read_row(read_tmp, input_orig, 8, orig_offset);
        // shift and copy to orig[i]
        orig[i] = spu_shuffle(read_tmp[orig_offset / 16],
                read_tmp[orig_offset / 16 + 1], mask[orig_offset % 16]);
        // jump to next row in input
        input_orig += w + orig_offset;
        orig_offset = input_orig % 128;
        input_orig -= orig_offset;

        // read 8 bytes
        read_row(read_tmp, input_orig, 8, orig_offset);
        // shift
        read_tmp[0] = spu_shuffle(read_tmp[orig_offset / 16],
                read_tmp[orig_offset / 16 + 1], mask[orig_offset % 16]);
        // merge
        orig[i] = spu_shuffle(orig[i], read_tmp[0], merge_mask);
        // jump to next row in input
        input_orig += w + orig_offset;
        orig_offset = input_orig % 128;
        input_orig -= orig_offset;
    }
}

void transpose_block(float *in_data, float *out_data)
{
    int i,j;
    for (i=0; i<8; ++i)
        for (j=0; j<8; ++j)
        {
            out_data[i*8+j] = in_data[j*8+i];
        }
}

void vec_transpose_block(VF *in_data, VF *out_data)
{
    transpose_block((float *)in_data, (float *)out_data);
}

void vec_dct_1d(VF *in_data, VF *out_data)
{
    int j;
    float *f_tmp;
    float *f_out = (float *)out_data;

    VF tmp[2];

    for (j=0; j<8; ++j)
    {
        tmp[0] = spu_madd(in_data[0], vec_dctlookup[j][0], vec_zero);
        tmp[1] = spu_madd(in_data[1], vec_dctlookup[j][1], tmp[0]);
        f_tmp = (float *)(&tmp[1]);
        f_out[j] = f_tmp[0] + f_tmp[1] + f_tmp[2] + f_tmp[3];
    }
}

static void vec_scale_block(VF *in_data, VF *out_data)
{
    out_data[0] = spu_madd(in_data[0], first_row, vec_zero);
    out_data[0] = spu_madd(out_data[0], first_column, vec_zero);
    out_data[1] = spu_madd(in_data[1], first_row, vec_zero);
    
    int i;
    for (i = 2; i < 16; i += 2)
    {
        out_data[i] = spu_madd(in_data[i], first_column, vec_zero);
    }
    for (i = 3; i < 16; i += 2)
    {
        out_data[i] = in_data[i];
    }
}

void quantize_block(float *in_data, float *out_data, uint8_t *quant_tbl)
{
    int zigzag;
    float ret;
    for (zigzag=0; zigzag < 64; ++zigzag)
    {
        uint8_t u = zigzag_U[zigzag];
        uint8_t v = zigzag_V[zigzag];

        float dct = in_data[v*8+u];

        /* Zig-zag and quantize */
        ret = (dct / 4.0) / quant_tbl[zigzag];
        ret = ret + 0.5f > (int)ret + 1 ? (int)ret + 1 : (int)ret; 
        out_data[zigzag] = ret;
    }
}

void vec_quantize_block(VF *in_data, VF *out_data, uint8_t *quant_tbl)
{
    quantize_block((float *)in_data, (float *)out_data, quant_tbl);
}

void dct_quant_block_8x8(ULL out_data, uint8_t *quant_tbl)
{
    VF mb[8*2];
    VF mb2[8*2];
    //VSS out_block[8];
    //int16_t *act_out;
    //float *act_mb;

    int i, v, tag = 1;

    for (i=0; i<16; ++i)
        mb2[i] = spu_convtf(block[i], 0);

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
    vec_quantize_block(mb, mb2, quant_tbl);

    /*
    act_mb = (float *)mb2;
    act_out = (int16_t *)out_block;
    for (i = 0; i < 64; ++i)
    {
        act_out[i] = (int16_t)act_mb[i];
    }
    */
    // send output
    spu_mfcdma64(mb2, mfc_ea2h(out_data), mfc_ea2l(out_data),
            8 * 2 * sizeof(VF), tag, MFC_PUT_CMD);
    spu_writech(MFC_WrTagMask, 1 << tag);
    spu_mfcstat(MFC_TAG_UPDATE_ALL);
}

void dct_get_block(dct_params_t *params)
{
    int i;
    VSI tmp[2];
    VUC *act_block;

    ULL src;
    int offset;
    for (i = 0; i < 8; ++i)
    {
        src = (ULL) (UL) (params->in_data + i * params->width);
        offset = src % 128;
        src -= offset;
        read_row(read_tmp, src, 8, offset);

        act_block = (VUC *)&block[i * 2];
        *act_block = spu_shuffle(read_tmp[offset / 16],
                        read_tmp[offset / 16 + 1], mask_dct1[offset % 16]);
        act_block = (VUC *)&block[i * 2 + 1];
        *act_block = spu_shuffle(read_tmp[offset / 16],
                        read_tmp[offset / 16 + 1], mask_dct2[offset % 16]);
        src = (ULL) (UL) (params->prediction + i * params->width);
        offset = src % 128;
        src -= offset;
        read_row(read_tmp, src, 8, offset);
        act_block = (VUC *)&tmp[0];
        *act_block = spu_shuffle(read_tmp[offset / 16],
                        read_tmp[offset / 16 + 1], mask_dct1[offset % 16]);
        act_block = (VUC *)&tmp[1];
        *act_block = spu_shuffle(read_tmp[offset / 16],
                        read_tmp[offset / 16 + 1], mask_dct2[offset % 16]);
        block[i * 2] = spu_sub(block[i * 2], tmp[0]);
        block[i * 2 + 1] = spu_sub(block[i * 2 + 1], tmp[1]);
    }
}

void print_spe(int *data)
{
    int i, j;
    printf("First block before DCT on SPE:\n");
    for (i = 0; i < 8; i++)
    {
        for (j = 0; j < 8; j++)
        {
            printf("%d ", data[i * 8 + j]);
        }
        printf("\n");
    }
    fflush(stdout);
}


int main(ULL spe, ULL argp, ULL envp) {

    prof_clear();

    // HACK
    int qp = (int)envp;
    if (qp > 0)
    {
        int i;
        for (i = 0; i < 64; ++i)
        {
            yquanttbl_def[i] = yquanttbl_def[i] / (qp / 10.0);
            uvquanttbl_def[i] = uvquanttbl_def[i] / (qp / 10.0);
        }
    }
    // ****

    sad_params_t params __attribute__((aligned(128)));
    dct_params_t dct_params __attribute__((aligned(128)));
    
    int tag = 1;
    int orig_offset;

    while(1) {
        orig = (VUC *) orig_array;
        ref = (VUC *) ref_array;
        read_tmp = (VUC *) read_tmp_array;

        // get task from mailbox
        mbox_data[0] = spu_read_in_mbox();
        mbox_data[1] = spu_read_in_mbox();
        mbox_data[2] = spu_read_in_mbox();
        mbox_data[3] = spu_read_in_mbox();

        if (mbox_data[0] == SPE_SAD)
        {
            prof_start();
            // GET params
            spu_mfcdma64(&params, mbox_data[2], mbox_data[3], sizeof(sad_params_t),
                    tag, MFC_GET_CMD);
            spu_writech(MFC_WrTagMask, 1 << tag);
            spu_mfcstat(MFC_TAG_UPDATE_ALL);
            
            // GET ints
            w = params.w;
            spe_nr = params.spe_nr;
            orig_offset = params.orig_offset;
            ref_offset = params.ref_offset;
            orig_x = params.orig_x;
            orig_y = params.orig_y;
            ref_w = params.ref_w;
            ref_h = params.ref_h;
            sad_w = ref_w - 7;
            sad_h = ref_h - 7;
            input_ref = params.ref;
            
            ds_init();
            
            // GET orig
            get_orig_input(params.orig, orig_offset);

            // calc
            ds(orig_x, orig_y);

            // PUT sad_out
            spu_mfcdma64(&sad_out, mfc_ea2h(params.sad_out), mfc_ea2l(params.sad_out),
                    sizeof(sad_out_t), tag, MFC_PUT_CMD);
            spu_writech(MFC_WrTagMask, 1 << tag);
            spu_mfcstat(MFC_TAG_UPDATE_ALL);
            
            // inform PPE about finish
            spu_write_out_intr_mbox ((unsigned) SPE_FINISH);
            prof_stop();
        }
        else if (mbox_data[0] == SPE_DCT)
        {
            uint8_t *quant_tbl;
            // GET params
            spu_mfcdma64(&dct_params, mbox_data[2], mbox_data[3], sizeof(dct_params_t),
                    tag, MFC_GET_CMD);
            spu_writech(MFC_WrTagMask, 1 << tag);
            spu_mfcstat(MFC_TAG_UPDATE_ALL);

            dct_get_block(&dct_params);
            //static int first = 1;
            //if (first)
            //{
            //    print_spe((int *)block);
            //    first = 0;
            //}

            if (dct_params.quantization == 0)
            {
                quant_tbl = yquanttbl_def;
            }
            else
            {
                quant_tbl = uvquanttbl_def;
            }

            dct_quant_block_8x8(dct_params.out_data, quant_tbl);
            
            // inform PPE about finish
            spu_write_out_intr_mbox ((unsigned) SPE_FINISH);
        }
        else // SPE_END
        {
            break;
        }
    }

    prof_write();
    return 0;
}
