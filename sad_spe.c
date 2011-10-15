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


#define spu_mfcdma64(ls, h, l, sz, tag, cmd) { \
        printf("spu_mfcdma64(%p, %x, %x, %lu, %d, %d) -- Line: %d\n", ls, h, l, sz, tag, cmd, __LINE__); \
    fflush(stdout); \
        spu_mfcdma64(ls, h, l, sz, tag, cmd); \
}


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

uint8_t mask_array[256] __attribute__((aligned(16)));

__vector unsigned char *mask = (__vector unsigned char *) mask_array;

__vector unsigned char merge_mask __attribute__((aligned(16)));

const int *big_diamond = big_diamond_array;
const int *small_diamond = small_diamond_array;

__vector unsigned char reg __attribute__((aligned(16)));
__vector unsigned char tmp __attribute__((aligned(16)));
__vector unsigned char sd __attribute__((aligned(16)));


void read_row(uint8_t *dst, ULL src, int size, int offset) {
    int tag = 1;
    int read_size = size + offset;
    read_size += (16 - (read_size%16)) % 16;
    spu_mfcdma64(dst, mfc_ea2h(src), mfc_ea2l(src), 
            (read_size) * sizeof(uint8_t), tag, MFC_GET_CMD);
    spu_writech(MFC_WrTagMask, 1 << tag);
    spu_mfcstat(MFC_TAG_UPDATE_ALL);
}

void get_ref(int x, int y) {
    int offset1 = (y*ref_w + x) % 16;
    int offset2 = (offset1 + ref_w) % 16;

    uint8_t *ref_ptr = &ref[y*ref_w + x - offset1];

    __vector unsigned char *ref_ptr1 = (__vector unsigned char *) ref_ptr;
    __vector unsigned char *ref_ptr2 = (__vector unsigned char *) (ref_ptr + 16);

    reg = spu_shuffle(*ref_ptr1, *ref_ptr2,  mask[offset1]);

    ref_ptr = &ref[(y+1)*ref_w + x - offset2];
    ref_ptr1 = (__vector unsigned char *) ref_ptr;
    ref_ptr2 = (__vector unsigned char *) (ref_ptr + 16);

    tmp = spu_shuffle(*ref_ptr1, *ref_ptr2, mask[offset2]);

    reg = spu_shuffle(reg, tmp, merge_mask);
}

int sad16(uint8_t *orig_reg_scalar) {
    __vector unsigned char orig_reg = 
        *((__vector unsigned char *) orig_reg_scalar);
    sd = spu_absd(orig_reg, reg);
    uint8_t *s = (uint8_t *) &sd;

    return (int) s[0] + (int) s[1] + (int) s[2] + (int) s[3] + (int) s[4] + (int) s[5] + (int) s[6] + (int) s[7]
        + (int) s[8] + (int) s[9] + (int) s[10] + (int) s[11] + (int) s[12] + (int) s[13] + (int) s[14] + (int) s[15];
}

int calc_sad(int x, int y) {
    int i, sum = 0;
    for (i = 0; i < 4; i++) {
        get_ref(x, y);
        sum += sad16(orig + 16*i);
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
}

void mask_init() {
    merge_mask = (__vector unsigned char) 
        {0,1,2,3,4,5,6,7,16,17,18,19,20,21,22,23};

    mask[0] = (__vector unsigned char) {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15};
    mask[1] = (__vector unsigned char) {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16};
    mask[2] = (__vector unsigned char) {2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17};
    mask[3] = (__vector unsigned char) {3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18};
    mask[4] = (__vector unsigned char) {4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19};
    mask[5] = (__vector unsigned char) {5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20};
    mask[6] = (__vector unsigned char) {6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21};
    mask[7] = (__vector unsigned char) {7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22};
    mask[8] = (__vector unsigned char) {8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23};
    mask[9] = (__vector unsigned char) {9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24};
    mask[10] = (__vector unsigned char) {10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25};
    mask[11] = (__vector unsigned char) {11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26};
    mask[12] = (__vector unsigned char) {12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27};
    mask[13] = (__vector unsigned char) {13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28};
    mask[14] = (__vector unsigned char) {14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29};
    mask[15] = (__vector unsigned char) {15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30};
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
    mask_init();
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

    printf("o=%x\to_x=%d\to_y=%d\to_o=%d\tr_o=%d\tr_w=%d\tr_h=%d\n",
            (unsigned) params.orig,orig_x,orig_y,orig_offset,ref_offset,
            ref_w,ref_h);

    
    // GET orig
    uint8_t read_tmp_array[16] __attribute__((aligned(16)));
    uint8_t *read_tmp = read_tmp_array;
    for (i = 0; i < ORIG_HEIGHT / 2; ++i) {
        int j;
        printf("0: params.orig = %x, orig_offset = %d\n", (unsigned) params.orig, orig_offset);
        fflush(stdout);
        read_row(orig, params.orig, 8, orig_offset);
        printf("1: params.orig = %x, orig_offset = %d\n", (unsigned) params.orig, orig_offset);
        fflush(stdout);
        params.orig += w + orig_offset;
        orig_offset = params.orig % 128;
        params.orig -= orig_offset;
        printf("2: params.orig = %x, orig_offset = %d\n", (unsigned) params.orig, orig_offset);
        fflush(stdout);
        
        read_row(read_tmp, params.orig, 8, orig_offset);
        printf("2.1: params.orig = %x, orig_offset = %d\n", (unsigned) params.orig, orig_offset);
        fflush(stdout);
        //TODO na wektory
        for (j = 0; j < 8; ++j)
            orig[8+j] = read_tmp[j];

        orig = (uint8_t *)orig + 2 * ORIG_WIDTH;
        params.orig += w + orig_offset;
        orig_offset = params.orig % 128;
        params.orig -= orig_offset;
        printf("3: params.orig = %x, orig_offset = %d\n", (unsigned) params.orig, orig_offset);
        fflush(stdout);
    }
    
    // GET ref
    for (i = 0; i < ref_h; ++i) {
        //read_row(ref, params.ref, ref_w, ref_offset);
        ref += REF_WIDTH;
        params.ref += w + ref_offset;
        ref_offset = params.ref % 128;
        params.ref -= ref_offset;
    }

    // calc
    //ds_init();
    //ds(orig_x, orig_y);

    // PUT sad_out
    /*spu_mfcdma64(&sad_out, mfc_ea2h(params.sad_out), mfc_ea2l(params.sad_out),
            sizeof(sad_out_t), tag, MFC_PUT_CMD);
    spu_writech(MFC_WrTagMask, 1 << tag);
    spu_mfcstat(MFC_TAG_UPDATE_ALL);
*/
    return 0;
}
