#include <stdio.h>
#include <spu_mfcio.h>
//#include <spu_intrinsics.h>
#include "spe.h"

#define REF_WIDTH 128
#define REF_HEIGHT 39
#define ORIG_WIDTH 16
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

uint8_t orig[ORIG_WIDTH * ORIG_HEIGHT] __attribute__((aligned(128)));
uint8_t ref[REF_WIDTH * REF_HEIGHT] __attribute__((aligned(128)));
int sad[ORIG_WIDTH * ORIG_HEIGHT] __attribute__((aligned(128)));

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

const int big_diamond[9][2] = {
    {0, 0},
    {2, 0},
    {1, 1},
    {0, 2},
    {-1, 1},
    {-2, 0},
    {-1, -1},
    {0, -2},
    {1, -1}
};

const int small_diamond[5][2] = {
    {0, 0},
    {1, 0},
    {0, 1},
    {-1, 0},
    {0, -1}
};

void read_row(uint8_t *dst, ULL src, int size, int offset) {
    int tag;
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

int calc_sad(int x, int y) {
    //TODO
    return 0; 
}

int get_sad(int x, int y, int *vec) {
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
    int best_dir;
    int sad;
    for (i = 0; i < 5; i++) {
        sad = get_sad(x, y, small_diamond[i]);
        if (sad < best_sad) {
            best_dir = i;
            best_sad = sad;
        }
    }
    sad_out.x = x + small_diamond[best_dir][0];
    sad_out.y = y + small_diamond[best_dir][1];
    sad_out.sad = best_sad;
}

void ds(int x, int y) {
    int i;
    int best_sad = INT_MAX;
    int best_dir;
    int sad;
    for (i = 0; i < 9; i++) {
        sad = get_sad(x, y, big_diamond[i]);
        if (sad < best_sad) {
            best_dir = i;
            best_sad = sad;
        }
    }
    if (best_dir == 0) {
        small_ds(x, y);
    }
    else {
        ds(x + big_diamond[best_dir][0], y + big_diamond[best_dir][1]);
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
        orig += ORIG_WIDTH;
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
        params.ref -= params_offset;
    }

    // calc
    ds();

    // PUT sad_out
    spu_mfcdma64(&sad_out, mfc_ea2h(params.sad_out), mfc_ea2l(params.sad_out),
            sizeof(sad_out_t), tag, MFC_PUT_CMD);
    spu_writech(MFC_WrTagMask, 1 << tag);
    spu_mfcstat(MFC_TAG_UPDATE_ALL);

    return 0;
}
