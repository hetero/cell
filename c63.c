#include "c63.h"

int working_spes;
int is_working[NUM_SPE];

spe_context_ptr_t spe[8] __attribute__((aligned(128)));
int mode;
pthread_mutex_t mutex;
pthread_t smart_thread[NUM_SPE];
pthread_cond_t main_cond, work_cond;

static sad_out_t spe_out[NUM_SPE] __attribute__((aligned(128)));
static sad_params_t sad_params[NUM_SPE] __attribute__((aligned(128)));
static thread_arg_t th_arg[NUM_SPE];

static float dct_out[NUM_SPE][8*8] __attribute__((aligned(128)));

static dct_params_t dct_params[NUM_SPE] __attribute__((aligned(128)));

int SPE_NUMBERS[6];

// SAD globals
int global_mb_x, global_mb_y, global_mb_rows, global_mb_cols, global_cc;
uint8_t *global_orig; 
uint8_t *global_ref;
struct c63_common *global_cm;

// DCT globals
int g_dct_row, g_dct_col, g_dct_width, g_dct_height, g_dct_quantization;
uint8_t *g_dct_in_data, *g_dct_prediction;
int16_t *g_dct_out_data;

void lock() {
    if (pthread_mutex_lock(&mutex) != 0)
        perror("lock failed");
}

void unlock() {
    if (pthread_mutex_unlock(&mutex) != 0)
        perror("unlock failed");
}

static void run_sad_spe(void *thread_arg) {
    thread_arg_t *arg = (thread_arg_t *) thread_arg;
    unsigned mbox_data[4];
    ULL params = (unsigned long) (arg->params);
    mbox_data[0] = SPE_SAD;
    mbox_data[1] = 0;
    mbox_data[2] = (unsigned) (params >> 32);
    mbox_data[3] = (unsigned) (params);
    
    sad_out_t *sad_out = (sad_out_t *) (unsigned long) (arg->params->sad_out);
    struct macroblock *mb = (struct macroblock *) (unsigned long) (arg->params->mb);

    // send params to SPE
    spe_in_mbox_write (arg->spe, mbox_data, 4, SPE_MBOX_ALL_BLOCKING);
    
    // wait for SPE's response
    spe_out_intr_mbox_read (arg->spe, mbox_data, 1, SPE_MBOX_ALL_BLOCKING);
    if (mbox_data[0] != SPE_FINISH) {
        printf("Unknown SPE response: %u\n", mbox_data[0]);
        perror("Unknown SPE response");
        exit(EXIT_FAILURE);
    }
    
    mb->mv_x = sad_out->x - arg->params->orig_x;
    mb->mv_y = sad_out->y - arg->params->orig_y;
    int best_sad = sad_out->sad;

    /* We only use motion vectors if the difference is small. */
    if (best_sad < 512)
    {
        mb->use_mv = 1;
//        printf("Using motion vector (%d, %d) with SAD %d\n", mb->mv_x, mb->mv_y, best_sad);
    }
    else
    {
        mb->use_mv = 0;
    }
}

static void run_dct_spe(int spe_nr, dct_params_t *dct_params)
{
    unsigned mbox_data[4];
    ULL params = (unsigned long) dct_params;
    mbox_data[0] = SPE_DCT;
    mbox_data[1] = 0;
    mbox_data[2] = (unsigned)(params >> 32);
    mbox_data[3] = (unsigned)params;

    spe_in_mbox_write(spe[spe_nr], mbox_data, 4, SPE_MBOX_ALL_BLOCKING);
    spe_out_intr_mbox_read(spe[spe_nr], mbox_data, 1, SPE_MBOX_ALL_BLOCKING);
}
    
/* Motion estimation for 8x8 block */
static void me_block_8x8(int spe_nr, struct c63_common *cm, int mb_x, int mb_y, uint8_t *orig, uint8_t *ref, int cc)
{
    /*printf("spe_nr = %d, orig = %x, mb_x = %d, mb_y = %d, cc = %d\n", spe_nr, (unsigned) orig, mb_x, mb_y, cc);
    fflush(stdout);*/
    
    struct macroblock *mb = &cm->curframe->mbs[cc][mb_y * cm->padw[cc]/8 + mb_x];

    int range = cm->me_search_range;

    int left = mb_x*8 - range;
    int top = mb_y*8 - range;
    int right = mb_x*8 + range;
    int bottom = mb_y*8 + range;

    int w = cm->padw[cc];
    int h = cm->padh[cc];

    /* Make sure we are within bounds of reference frame */
    // TODO: Support partial frame bounds
    if (left < 0)
        left = 0;
    if (top < 0)
        top = 0;
    if (right > (w - 8))
        right = w - 8;
    if (bottom > (h - 8))
        bottom = h - 8;

    int mx = mb_x * 8;
    int my = mb_y * 8;
    
    // pointer to orig block
    uint8_t *orig_ptr = orig + my*w + mx;
    sad_params[spe_nr].orig_offset = (unsigned long) orig_ptr % 128;
    orig_ptr -= sad_params[spe_nr].orig_offset;
    sad_params[spe_nr].orig_x = mx - left;
    sad_params[spe_nr].orig_y = my - top;

    // pointer to ref window
    uint8_t *ref_ptr = ref + top*w + left;
    sad_params[spe_nr].ref_offset = (unsigned long) ref_ptr % 128;
    ref_ptr -= sad_params[spe_nr].ref_offset;
    sad_params[spe_nr].ref_w = right - left + 7;
    sad_params[spe_nr].ref_h = bottom - top + 7;

    sad_params[spe_nr].orig = (unsigned long) orig_ptr;
    sad_params[spe_nr].ref = (unsigned long) ref_ptr;
    sad_params[spe_nr].sad_out = (unsigned long) &spe_out[spe_nr];
    sad_params[spe_nr].mb = (unsigned long) mb;
    sad_params[spe_nr].w = w;

    th_arg[spe_nr].spe = spe[spe_nr];
    th_arg[spe_nr].params = &sad_params[spe_nr];
    
    run_sad_spe(&th_arg[spe_nr]);
}


void print_block(signed short *data)
{
    int i, j;
    printf("First block after DCT:\n");
    for (i = 0; i < 8; i++)
    {
        for (j = 0; j < 8; j++)
        {
            printf("%d ", data[i * 8 + j]);
        }
        printf("\n");
    }
}


static void dct_block_8x8(int spe_nr, int width, int height, int row, int col, uint8_t *in_data, uint8_t *prediction, int16_t *out_data, int quantization) 
{
    int r, c;
    int16_t *act_out;
    uint8_t *ptr = in_data + row * width + col;
    dct_params[spe_nr].in_data = (unsigned long) ptr;

    ptr = prediction + row * width + col;
    dct_params[spe_nr].prediction = (unsigned long) ptr;

    // aligned memory for sure
    //int16_t *ptr_16 = out_data + row * width + col * 8;
    dct_params[spe_nr].out_data = (unsigned long) dct_out[spe_nr];

    dct_params[spe_nr].quantization = quantization;

    dct_params[spe_nr].width = width;

    run_dct_spe(spe_nr, &dct_params[spe_nr]);
//    if (row == 0 && col == 0)
//        print_block(out_data);
    int16_t *ptr_16 = out_data + row * width + col * 8;
    for (r = 0; r < 64; ++r)
        ptr_16[r] = (int16_t)(dct_out[spe_nr][r]);
}

void *run_smart_thread(void *void_spe_nr) {
    int spe_nr = *(int *) void_spe_nr;
    while (1) {
        lock();
        while (mode == WAIT_MODE) {
            if (pthread_cond_wait(&work_cond, &mutex) != 0)
                perror ("cond wait failed");
        }

        if (mode == OFF_MODE) {
            unlock();
            break;
        }

        if (mode == ENDING_JOB_MODE) {
            if (is_working[spe_nr]) {
                is_working[spe_nr] = 0;
                working_spes--;
                if (working_spes == 0) {
                    mode = WAIT_MODE;
                    if (pthread_cond_signal(&main_cond) != 0)
                        perror ("cond signal failed");
                }
            }
            unlock();
            continue;
        }

        if ((mode == SAD_MODE || mode == DCT_MODE || mode == IDCT_MODE)
                && !is_working[spe_nr]) { 
            working_spes++;
            is_working[spe_nr] = 1;
        }

        if (mode == SAD_MODE)
        {
            // coords of block
            int mb_x = global_mb_x;
            int mb_y = global_mb_y;
            // Y/U/V
            int cc = global_cc;
            uint8_t *orig = global_orig;
            uint8_t *ref = global_ref;
            struct c63_common *cm = global_cm;

            if (mb_x < global_mb_cols && mb_y < global_mb_rows) {
                global_mb_x++;
                if (global_mb_x >= global_mb_cols) {
                    global_mb_x = 0;
                    global_mb_y++;
                }
                unlock();
            
                me_block_8x8(spe_nr, cm, mb_x, mb_y, orig, ref, cc);
            }
            else {
                global_mb_x = 0;
                global_mb_y = 0;

                if (global_cc == 0) {
                    global_orig = cm->curframe->orig->U;
                    global_ref = cm->refframe->recons->U;
                    global_mb_cols /= 2;
                    global_mb_rows /= 2;
                }
                else if (global_cc == 1) {
                    global_orig = cm->curframe->orig->V;
                    global_ref = cm->refframe->recons->V;
                }
                else { // global_cc == 2
                    mode = ENDING_JOB_MODE;
                }
                global_cc = (global_cc + 1) % 3;
                
                unlock();
            }
        }
        else if (mode == DCT_MODE)
        {
            int row = g_dct_row;
            int col = g_dct_col;
            int width = g_dct_width;
            int height = g_dct_height;
            uint8_t *in_data = g_dct_in_data;
            uint8_t *prediction = g_dct_prediction;
            int quantization = g_dct_quantization;
            int16_t *out_data = g_dct_out_data;

            if (row < height && col < width)
            {
                g_dct_col += 8;
                if (g_dct_col >= width)
                {
                    g_dct_col = 0;
                    g_dct_row += 8;
                }
                unlock();
                
                dct_block_8x8(spe_nr, width, height, row, col, in_data, prediction, out_data, quantization); 
            }
            else
            {
                mode = ENDING_JOB_MODE;
                unlock();
            }
        }
    }

    return NULL;
}

