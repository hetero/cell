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

/*
#define prof_cp(cp_num) \
{ \
    if (_counting) { \
        unsigned int end = spu_readch(SPU_RdDec); \
        _count += (end > _count_base) ? (DECR_MAX + _count_base - end) : \
            (_count_base - end); \
    } \
    printf("SPU#: CP"#cp_num", %llu\n", _count); \
    _count_base = spu_readch(SPU_RdDec); \
}
*/

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

#define prof_write() printf("SPU#: %3.3f\n", (float) _count / 1e6)

float total_time = 0;

uint8_t orig_array[ORIG_WIDTH * ORIG_HEIGHT] __attribute__((aligned(128)));
uint8_t ref_array[REF_WIDTH * REF_HEIGHT] __attribute__((aligned(128)));
uint8_t read_tmp_array[256] __attribute__((aligned(128)));
int sad[SAD_WIDTH * SAD_HEIGHT] __attribute__((aligned(128)));

sad_out_t sad_out __attribute__((aligned(128)));

VUC *orig;
VUC *ref;
VUC *read_tmp;

unsigned mbox_data[4];
int w, wm128, orig_offset, ref_offset, orig_x, orig_y, ref_w, ref_h, sad_w, sad_h;

VUC reg;
VUC tmp;
VUC sd;

/*
void start() {
    clock_gettime(CLOCK_REALTIME, &start_timer);
}

void stop() {
    clock_gettime(CLOCK_REALTIME, &stop_timer);
    total_time += stop_timer.tv_sec - start_timer.tv_sec + 
        (float) (stop_timer.tv_nsec - start_timer.tv_nsec) / 1e9;
}

void print_time() {
    printf("SPE time: %2.3f\n", total_time);
}
*/

void read_row(VUC *dst, ULL src, int size, int offset) {
    int tag = 1;
    int read_size = size + offset;
    read_size += (16 - (read_size%16)) % 16;
    spu_mfcdma64(read_tmp, mfc_ea2h(src), mfc_ea2l(src), 
            (read_size) * sizeof(uint8_t), tag, MFC_GET_CMD);
    spu_writech(MFC_WrTagMask, 1 << tag);
    spu_mfcstat(MFC_TAG_UPDATE_ALL);
}

// stores into "reg" two rows of ref beginnig from x,y
// assumption : REF_WIDTH % 16 = 0
void get_ref(int x, int y) {
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
    prof_clear();

    sad_params_t params __attribute__((aligned(128)));
    int tag = 1, i, j;

    while(1) {
        orig = (VUC *) orig_array;
        ref = (VUC *) ref_array;
        read_tmp = (VUC *) read_tmp_array;

        prof_start();
        mbox_data[0] = spu_read_in_mbox();
        mbox_data[1] = spu_read_in_mbox();
        mbox_data[2] = spu_read_in_mbox();
        mbox_data[3] = spu_read_in_mbox();
        prof_stop();

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
        

        // GET orig (two rows in each loop step)
        for (i = 0; i < ORIG_HEIGHT / 2; ++i) {
            // read 8 bytes and everything on the left to 128 boundary
            read_row(read_tmp, params.orig, 8, orig_offset);
            // shift and copy to orig[i]
            orig[i] = spu_shuffle(read_tmp[orig_offset / 8],
                    read_tmp[orig_offset / 8 + 1], mask[orig_offset % 8]);
            // jump to next row in input
            params.orig += w + orig_offset;
            orig_offset = params.orig % 128;
            params.orig -= orig_offset;

            // read 8 bytes
            read_row(read_tmp, params.orig, 8, orig_offset);
            // shift
            read_tmp[0] = spu_shuffle(read_tmp[orig_offset / 8],
                    read_tmp[orig_offset / 8 + 1], mask[orig_offset % 8]);
            // merge
            orig[i] = spu_shuffle(orig[i], read_tmp[0], merge_mask);
            // jump to next row in input
            params.orig += w + orig_offset;
            orig_offset = params.orig % 128;
            params.orig -= orig_offset;
        }
        
        // GET ref
        int vectors = ref_w / 16 + (ref_w % 16 > 0);
        for (i = 0; i < ref_h; ++i) {
            // read row of input (and shit on the left)
            read_row(read_tmp, params.ref, ref_w, ref_offset);
            // shift block by block
            for (j = 0; j < vectors; ++j) {
                // first block (8) in vector (16)
                ref[i * VEC_REF_WIDTH + j] = spu_shuffle(
                        read_tmp[ref_offset / 8 + j],
                        read_tmp[ref_offset / 8 + j + 1],
                        mask[ref_offset % 8]);
                // second block (8) in vector (16)
                read_tmp[0] = spu_shuffle(
                        read_tmp[ref_offset / 8 + j + 1],
                        read_tmp[ref_offset / 8 + j + 2],
                        mask[ref_offset % 8]);
                // merge
                ref[i * VEC_REF_WIDTH + j] = spu_shuffle(
                        ref[i * VEC_REF_WIDTH + j],
                        read_tmp[0],
                        merge_mask);
            }
            // jump to next row in input
            params.ref += w + ref_offset;
            ref_offset = params.ref % 128;
            params.ref -= ref_offset;
        }

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

    prof_write();
    return 0;
}
