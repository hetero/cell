#include <stdio.h>
#include <spu_mfcio.h>
#include <spu_intrinsics.h>
#include "spe.h"

#define REF_WIDTH 128
#define REF_HEIGHT 39
#define ORIG_WIDTH 8
#define ORIG_HEIGHT 8
#define SAD_WIDTH 32
#define SAD_HEIGHT 32

/*
#define spu_mfcdma64(ls, h, l, sz, tag, cmd) { \
        printf("spu_mfcdma64(%p, %x, %x, %lu, %d, %d) -- Line: %d\n", ls, h, l, sz, tag, cmd, __LINE__); \
        spu_mfcdma64(ls, h, l, sz, tag, cmd); \
}
*/

typedef unsigned long long ULL;

uint8_t orig_array[ORIG_WIDTH * ORIG_HEIGHT] __attribute__((aligned(128)));
uint8_t ref_array[REF_WIDTH * REF_HEIGHT] __attribute__((aligned(128)));
int sad[ORIG_WIDTH * ORIG_HEIGHT] __attribute__((aligned(128)));
uint8_t *orig = orig_array;
uint8_t *ref = ref_array;

sad_out_t sad_out __attribute__((aligned(128)));

int w __attribute__((aligned(128)));
int wm128 __attribute__((aligned(128)));
int orig_offset __attribute__((aligned(128)));
int ref_offset __attribute__((aligned(128)));
int orig_x __attribute__((aligned(128)));
int orig_y __attribute__((aligned(128)));
int ref_w __attribute__((aligned(128)));
int ref_h __attribute__((aligned(128)));
int sad_w __attribute__((aligned(128)));
int sad_h __attribute__((aligned(128)));

const int big_diamond_array[9 * 2] = {
    0, 0,
    2, 0,
    1, 1,
    0, 2,
    -1, 1,
    -2, 0,
    -1, -1,
    0, -2,
    1, -1
};

const int small_diamond_array[5 * 2] = {
    0, 0,
    1, 0,
    0, 1,
    -1, 0,
    0, -1
};

const int *big_diamond = big_diamond_array;
const int *small_diamond = small_diamond_array;

__vector unsigned char reg __((aligned(16)));
__vector unsigned char tmp __((aligned(16)));

void read_row(uint8_t *dst, ULL src, int size, int offset) {
    int tag = 1;
    int read_size = size + offset;
    read_size += (16 - (read_size%16)) % 16;
    spu_mfcdma64(dst, mfc_ea2h(src), mfc_ea2l(src), 
            (read_size) * sizeof(uint8_t), tag, MFC_GET_CMD);
    spu_writech(MFC_WrTagMask, 1 << tag);
    spu_mfcstat(MFC_TAG_UPDATE_ALL);
}

void spe_sad_block_8x8(uint8_t *block1, uint8_t *block2, int stride, int *result)
{   
    *result = 0;

    int u,v;
    for (v=0; v<8; ++v)
        for (u=0; u<8; ++u) {
            if (block2[v*stride+u] >= block1[v*stride+u])
                result += block2[v*stride+u] - block1[v*stride+u];
            else
                result += block1[v*stride+u] - block2[v*stride+u];
        }
}

void get_ref(int x, int y) {
    int offset1 = (y*wm128 + x) % 16;
    int offset2 = (offset1 + wm128) % 16;

    uint8_t *ref_ptr = &ref[

    if (offset1 <= 8) {
        tmp = spu_slqwbyte(


int calc_sad(int x, int y) {
    int i, sum = 0;
    for (i = 0; i < 4; i++) {
        get_ref(x, y);
        sum += sad16((__vector unsigned char) (orig + 16*i));
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
}

void small_ds(int x, int y) {
    int i;
    int best_sad = INT_MAX;
    int best_dir = 0;
    int sad;
    for (i = 0; i < 5; i++) {
        sad = get_sad(x, y, &small_diamond[2 * i]);
        if (sad < best_sad) {
            best_dir = i;
            best_sad = sad;
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
    int sad;
    for (i = 0; i < 9; i++) {
        sad = get_sad(x, y, &big_diamond[2 * i]);
        if (sad < best_sad) {
            best_dir = i;
            best_sad = sad;
        }
    }
    if (best_dir == 0) {
        small_ds(x, y);
    }
    else {
        ds(x + big_diamond[2 * best_dir], y + big_diamond[2 * best_dir]);
    }
}

int main(ULL spe, ULL argp, ULL envp) {
    int tag = 1, i;
    sad_params_t params __attribute__((aligned(128)));
    
    // GET params
    spu_mfcdma64(&params, mfc_ea2h(argp), mfc_ea2l(argp), sizeof(sad_params_t),
            tag, MFC_GET_CMD);
    spu_writech(MFC_WrTagMask, 1 << tag);
    spu_mfcstat(MFC_TAG_UPDATE_ALL);
    
    // GET ints
    w = params.w;
    wm128 = params.wm128;
    orig_offset = params.orig_offset;
    ref_offset = params.ref_offset;
    orig_x = params.orig_x;
    orig_y = params.orig_y;
    ref_w = params.ref_w;
    ref_h = params.ref_w;
    sad_w = ref_w - 7;
    sad_h = ref_h - 7;
    
    // GET orig
    for (i = 0; i < ORIG_HEIGHT; ++i) {
        read_row(orig, params.orig, 8, orig_offset);
        orig = (uint8_t *)orig + ORIG_WIDTH;
        params.orig += w + orig_offset;
        orig_offset = params.orig % 128;
        params.orig -= orig_offset;
    }
    
    // GET ref
    for (i = 0; i < ref_h; ++i) {
        read_row(ref, params.ref, ref_w, ref_offset);
        ref += REF_WIDTH;
        params.ref += w + ref_offset;
        ref_offset = params.ref % 128;
        params.ref -= ref_offset;
    }

    // calc
    ds(orig_x, orig_y);

    // PUT sad_out
    spu_mfcdma64(&sad_out, mfc_ea2h(params.sad_out), mfc_ea2l(params.sad_out),
            sizeof(sad_out_t), tag, MFC_PUT_CMD);
    spu_writech(MFC_WrTagMask, 1 << tag);
    spu_mfcstat(MFC_TAG_UPDATE_ALL);

    return 0;
}
