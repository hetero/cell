#include <stdio.h>
#include <spu_mfcio.h>
#include <spu_intrinsics.h>
#include "spe.h"
#include "spe_only.h"

#define REF_WIDTH 39
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

uint32_t mbox_data[4] __attribute__((aligned(128)));

uint8_t orig_array[ORIG_WIDTH * ORIG_HEIGHT] __attribute__((aligned(128)));
uint8_t ref_array[REF_WIDTH * REF_HEIGHT] __attribute__((aligned(128)));
int sad[SAD_WIDTH * SAD_HEIGHT] __attribute__((aligned(128)));
uint8_t *orig;
uint8_t *ref;
uint8_t read_tmp_array[256] __attribute__((aligned(128)));
uint8_t *read_tmp;
sad_params_t params __attribute__((aligned(128)));

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

const int *big_diamond = big_diamond_array;
const int *small_diamond = small_diamond_array;

VUC reg __attribute__((aligned(16)));
VUC tmp __attribute__((aligned(16)));
VUC sd __attribute__((aligned(16)));

void read_row(uint8_t *dst, ULL src, int size, int offset) {
    int i, tag = 1;
    int read_size = size + offset;
    read_size += (16 - (read_size%16)) % 16;
    spu_mfcdma64(read_tmp, mfc_ea2h(src), mfc_ea2l(src), 
            (read_size) * sizeof(uint8_t), tag, MFC_GET_CMD);
    spu_writech(MFC_WrTagMask, 1 << tag);
    spu_mfcstat(MFC_TAG_UPDATE_ALL);
    for (i = 0; i < size; ++i)
        dst[i] = read_tmp[offset + i];
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
    int i;
    int sum = 0;
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

int main(ULL spe, ULL argp, ULL envp) {
    int tag = 1;
    while(1) {
        orig = orig_array;
        ref = ref_array;
        read_tmp = read_tmp_array;
        mbox_data[0] = spu_read_in_mbox();
        mbox_data[1] = spu_read_in_mbox();
        mbox_data[2] = spu_read_in_mbox();
        mbox_data[3] = spu_read_in_mbox();

        if (mbox_data[0] == SPE_END)
            break;

        // GET params
        spu_mfcdma64(&params, mbox_data[2], mbox_data[3], sizeof(sad_params_t),
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
        ref_h = params.ref_h;
        sad_w = ref_w - 7;
        sad_h = ref_h - 7;

        // GET orig
        int i;
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
            ref += ref_w;
            params.ref += w + ref_offset;
            ref_offset = params.ref % 128;
            params.ref -= ref_offset;
        }
        ref = ref_array;
        orig = orig_array;
        read_tmp = read_tmp_array;

        // calc
        ds_init();
        ds(orig_x, orig_y);

        // PUT sad_out
        spu_mfcdma64(&sad_out, mfc_ea2h(params.sad_out), mfc_ea2l(params.sad_out),
                sizeof(sad_out_t), tag, MFC_PUT_CMD);
        spu_writech(MFC_WrTagMask, 1 << tag);
        spu_mfcstat(MFC_TAG_UPDATE_ALL);
        
        // inform PPE about finish
        spu_write_out_intr_mbox ((unsigned) SPE_FINISH);
    }
    return 0;
}
