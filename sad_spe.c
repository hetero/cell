#include <stdio.h>
#include <spu_mfcio.h>
//#include <spu_intrinsics.h>
#include "spe.h"

#define MAX_WIDTH 3840
// memory usage ~ 120 kB

/*
#define spu_mfcdma64(ls, h, l, sz, tag, cmd) { \
        printf("spu_mfcdma64(%p, %x, %x, %lu, %d, %d) -- Line: %d\n", ls, h, l, sz, tag, cmd, __LINE__); \
        spu_mfcdma64(ls, h, l, sz, tag, cmd); \
}
*/

typedef unsigned long long ULL;

uint8_t orig[4*MAX_WIDTH] __attribute__((aligned(128)));
uint8_t ref[4*MAX_WIDTH] __attribute__((aligned(128)));
sad_out_t sad_out __attribute__((aligned(128)));
int w __attribute__((aligned(128)));
int shift __attribute__((aligned(128)));

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

void spe_sad_4rows()
{   
    int x, y;
    int best_rows_sad = INT_MAX;
    int rows_sad, best_rows_x = 0, best_rows_y = 0;
    for (y = 0; y < 4; y++)
    {   
        for (x = 0; x < 32; x++)
        {   
            spe_sad_block_8x8(orig + shift, ref + y*w + x + shift, w, &rows_sad);
            if (rows_sad < best_rows_sad)
            {   
                best_rows_x = x;
                best_rows_y = y;
                best_rows_sad = rows_sad;
            }
        }
    }
    sad_out.sad = best_rows_sad;
    sad_out.x = best_rows_x;
    sad_out.y = best_rows_y;
}

int main(ULL spe, ULL argp, ULL envp) {
    int tag = 1;
    sad_params_t params __attribute__((aligned(128)));
    
    // GET params
    spu_mfcdma64(&params, mfc_ea2h(argp), mfc_ea2l(argp), sizeof(sad_params_t),
            tag, MFC_GET_CMD);
    spu_writech(MFC_WrTagMask, 1 << tag);
    spu_mfcstat(MFC_TAG_UPDATE_ALL);
    
    // GET w, shift
    w = params.w;
    shift = params.shift;
    
    // GET orig
    spu_mfcdma64(orig, mfc_ea2h(params.orig), mfc_ea2l(params.orig), 
            (3*w + 32 + 2*shift) * sizeof(uint8_t), tag, MFC_GET_CMD);
    spu_writech(MFC_WrTagMask, 1 << tag);
    spu_mfcstat(MFC_TAG_UPDATE_ALL);
    
    // GET ref
    spu_mfcdma64(ref, mfc_ea2h(params.ref), mfc_ea2l(params.ref), 
            (3*w + 32 + 2*shift) * sizeof(uint8_t), tag, MFC_GET_CMD);
    spu_writech(MFC_WrTagMask, 1 << tag);
    spu_mfcstat(MFC_TAG_UPDATE_ALL);

    // calc
    spe_sad_4rows();

    // PUT sad_out
    spu_mfcdma64(&sad_out, mfc_ea2h(params.out), mfc_ea2l(params.out), 
             sizeof(sad_out_t), tag, MFC_PUT_CMD);
    spu_writech(MFC_WrTagMask, 1 << tag);
    spu_mfcstat(MFC_TAG_UPDATE_ALL);

    return 0;
}
