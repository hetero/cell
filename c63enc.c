#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <getopt.h>
#include <errno.h>
#include <stdint.h>
#include <math.h>
#include <assert.h>
#include <limits.h>
#include <pthread.h>
#include <time.h>

#include "c63.h"
#include "tables.h"
#include "ppe.h"


static char *output_file, *input_file;
FILE *outfile;
//FILE *predfile;

static int limit_numframes = 0;

static uint32_t width;
static uint32_t height;
static uint32_t yph;
static uint32_t ypw;
static uint32_t uph;
static uint32_t upw;
static uint32_t vph;
static uint32_t vpw;

/* getopt */
extern int optind;
extern char *optarg;

spe_program_handle_t *prog;
pthread_t run_thread[NUM_SPE];
thread_arg_t run_arg[NUM_SPE];

struct timespec start_time, stop_time;

float total_time = 0;

void start() {
    clock_gettime(CLOCK_REALTIME, &start_time);
}

void stop() {
    clock_gettime(CLOCK_REALTIME, &stop_time);
    total_time += stop_time.tv_sec - start_time.tv_sec + 
        (stop_time.tv_nsec - start_time.tv_nsec) / 1e9;
}

void print_time() {
    printf("PPE time: %f\n", total_time);
}

/* Read YUV frames */
static yuv_t* read_yuv(FILE *file)
{
    int nothing;
    size_t len = 0;
    yuv_t *image = NULL;
    nothing = posix_memalign((void **) &image, 128, sizeof(yuv_t));

    printf("Reading...\n");

    /* Read Y' */
    nothing = posix_memalign((void **) &(image->Y), 128, width*(height + 8));
    len += fread(image->Y, 1, width*height, file);
    if(ferror(file))
    {
        perror("ferror");
        exit(EXIT_FAILURE);
    }

    /* Read U */
    nothing = posix_memalign((void **) &(image->U), 128, width*height);
    len += fread(image->U, 1, (width*height)/4, file);
    if(ferror(file))
    {
        perror("ferror");
        exit(EXIT_FAILURE);
    }

    /* Read V */
    nothing = posix_memalign((void **) &(image->V), 128, width*height);
    len += fread(image->V, 1, (width*height)/4, file);
    if(ferror(file))
    {
        perror("ferror");
        exit(EXIT_FAILURE);
    }

    if(len != width*height*1.5)
    {
        fprintf(stderr, "Reached end of file.\n");
        return NULL;
    }

    return image;
}

void *run_spe(void *thread_arg) {
    int ret;
    thread_arg_t *arg = (thread_arg_t *) thread_arg;
    unsigned int entry;
    spe_stop_info_t stop_info;

    entry = SPE_DEFAULT_ENTRY;
    printf("Running SPE...\n");
    ret = spe_context_run(arg->spe, &entry, 0, NULL, (void *) (unsigned long) arg->qp, &stop_info);
    if (ret < 0) {
        perror("spe_context_run");
        return NULL;
    }

    return NULL;
}

void spe_init(uint8_t qp) {
    working_spes = 0;

    int i, ret;
    prog = spe_image_open("sad_spe.elf");
    if (!prog) {
        perror("spe_image_open");
        exit(1);
    }   

    for (i = 0; i < NUM_SPE; i++) {
        is_working[i] = 0;
        spe[i] = spe_context_create(0, NULL);
        if (!spe[i]) {
            perror("spe_context_create");
            exit(1);
        }   

        ret = spe_program_load(spe[i], prog);
        if (ret) {
            perror("spe_program_load");
            exit(1);
        }
        run_arg[i].spe = spe[i];
        run_arg[i].qp = qp;
        ret = pthread_create(&run_thread[i], NULL, run_spe, &run_arg[i]);
        if (ret) {
            perror("run_pthread_create");
            exit(1);
        }   
    }   
}

void spe_dispose() {    
    int i, ret;
    for (i = 0; i < NUM_SPE; i++) {
        unsigned int mbox_data[4];
        mbox_data[0] = SPE_END;
        mbox_data[1] = mbox_data[2] = mbox_data[3] = 0;
        spe_in_mbox_write(spe[i], mbox_data, 4, SPE_MBOX_ALL_BLOCKING);

        pthread_join(run_thread[i], NULL);
        ret = spe_context_destroy(spe[i]);
        if (ret) {
            perror("spe_context_destroy");
            exit(1);
        }
    }

    ret = spe_image_close(prog);
    if (ret) {
        perror("spe_image_close");
        exit(1);
    }
}

static void c63_encode_image(struct c63_common *cm, yuv_t *image)
{
    /* Advance to next frame */
    destroy_frame(cm->refframe);
    cm->refframe = cm->curframe;
    cm->curframe = create_frame(cm, image);

    /* Check if keyframe */
    if (cm->framenum == 0 || cm->frames_since_keyframe == cm->keyframe_interval)
    {
        cm->curframe->keyframe = 1;
        cm->frames_since_keyframe = 0;

        fprintf(stderr, " (keyframe) ");
    }
    else
        cm->curframe->keyframe = 0;

    if (!cm->curframe->keyframe)
    {
        /* Motion Estimation */
        c63_motion_estimate(cm);
        
        /* Motion Compensation */
        c63_motion_compensate(cm);
    }

    /* DCT and Quantization */
    dct_quantize(image->Y, cm->curframe->predicted->Y, cm->padw[0], cm->padh[0], cm->curframe->residuals->Ydct, cm->quanttbl[0], 0);
    dct_quantize(image->U, cm->curframe->predicted->U, cm->padw[1], cm->padh[1], cm->curframe->residuals->Udct, cm->quanttbl[1], 1);
    dct_quantize(image->V, cm->curframe->predicted->V, cm->padw[2], cm->padh[2], cm->curframe->residuals->Vdct, cm->quanttbl[2], 2);

    /* Reconstruct frame for inter-prediction */
    dequantize_idct(cm->curframe->residuals->Ydct, cm->curframe->predicted->Y, cm->ypw, cm->yph, cm->curframe->recons->Y, cm->quanttbl[0]);
    dequantize_idct(cm->curframe->residuals->Udct, cm->curframe->predicted->U, cm->upw, cm->uph, cm->curframe->recons->U, cm->quanttbl[1]);
    dequantize_idct(cm->curframe->residuals->Vdct, cm->curframe->predicted->V, cm->vpw, cm->vph, cm->curframe->recons->V, cm->quanttbl[2]);

    /* dump_image can be used here to check if the prediction is correct. */
    //dump_image(cm->curframe->predicted, cm->width, cm->height, predfile);

    start();
    write_frame(cm);
    stop();

    ++cm->framenum;
    ++cm->frames_since_keyframe;
}

struct c63_common* init_c63_enc(int width, int height)
{
    int i;
    struct c63_common *cm = calloc(1, sizeof(struct c63_common));

    cm->width = width;
    cm->height = height;
    cm->padw[0] = cm->ypw = (uint32_t)(ceil(width/16.0f)*16);
    cm->padh[0] = cm->yph = (uint32_t)(ceil(height/16.0f)*16);
    cm->padw[1] = cm->upw = (uint32_t)(ceil(width*UX/(YX*8.0f))*8);
    cm->padh[1] = cm->uph = (uint32_t)(ceil(height*UY/(YY*8.0f))*8);
    cm->padw[2] = cm->vpw = (uint32_t)(ceil(width*VX/(YX*8.0f))*8);
    cm->padh[2] = cm->vph = (uint32_t)(ceil(height*VY/(YY*8.0f))*8);

    cm->mb_cols = cm->ypw / 8;
    cm->mb_rows = cm->yph / 8;


    /* Quality parameters */
    cm->qp = 25;                 // Constant quantization factor. Range: [1..50]
    cm->me_search_range = 16;    // Pixels in every direction
    cm->keyframe_interval = 100;  // Distance between keyframes


    /* Initialize quantization tables */
    for (i=0; i<64; ++i)
    {
        cm->quanttbl[0][i] = yquanttbl_def[i] / (cm->qp / 10.0);
        cm->quanttbl[1][i] = uvquanttbl_def[i] / (cm->qp / 10.0);
        cm->quanttbl[2][i] = uvquanttbl_def[i] / (cm->qp / 10.0);
    }

    return cm;
}

static void print_help()
{
    fprintf(stderr, "Usage: ./c63enc [options] input_file\n");
    fprintf(stderr, "Commandline options:\n");
    fprintf(stderr, "  -h                             height of images to compress\n");
    fprintf(stderr, "  -w                             width of images to compress\n");
    fprintf(stderr, "  -o                             Output file (.mjpg)\n");
    fprintf(stderr, "  [-f]                           Limit number of frames to encode\n");
    fprintf(stderr, "\n");

    exit(EXIT_FAILURE);
}

int main(int argc, char **argv)
{
    int c;

    for (c = 0; c < 6; ++c)
    {
       SPE_NUMBERS[c] = c; 
    }

    yuv_t *image;

    if(argc == 1)
    {
        print_help();
    }

    while((c = getopt(argc, argv, "h:w:o:f:i:")) != -1)
    {
        switch(c)
        {
        case 'h':
            height = atoi(optarg);
            break;
        case 'w':
            width = atoi(optarg);
            break;
        case 'o':
            output_file = optarg;
            break;
        case 'f':
            limit_numframes = atoi(optarg);
            break;
        default:
            print_help();
            break;
        }
    }


    if(optind >= argc)
    {
        fprintf(stderr, "Error getting program options, try --help.\n");
        exit(EXIT_FAILURE);
    }

    outfile = fopen(output_file, "wb");
    if(outfile == NULL)
    {
        perror("fopen");
        exit(EXIT_FAILURE);
    }

//    predfile = fopen("/tmp/pred.yuv", "wb");
    
    struct c63_common *cm = init_c63_enc(width, height);
    cm->e_ctx.fp = outfile;

    spe_init(cm->qp);
    if (pthread_mutex_init(&mutex, 0) != 0)
        perror("Mutex init failed.");
    
    lock();

    int i;
    int spe_nr = rand() % NUM_SPE;
    for (i = 0; i < NUM_SPE; i++) {
        int ret = pthread_create(&smart_thread[spe_nr], NULL, run_smart_thread,                &SPE_NUMBERS[spe_nr]);
        if (ret) { 
            perror("pthread_create");
            exit(1);
        }       
        spe_nr = (spe_nr + 1) % NUM_SPE;
    }


    /* Calculate the padded width and height */
    ypw = (uint32_t)(ceil(width/8.0f)*8);
    yph = (uint32_t)(ceil(height/8.0f)*8);
    upw = (uint32_t)(ceil(width*UX/(YX*8.0f))*8);
    uph = (uint32_t)(ceil(height*UY/(YY*8.0f))*8);
    vpw = (uint32_t)(ceil(width*VX/(YX*8.0f))*8);
    vph = (uint32_t)(ceil(height*VY/(YY*8.0f))*8);

    input_file = argv[optind];

    if (limit_numframes)
        fprintf(stderr, "Limited to %d frames.\n", limit_numframes);

    FILE *infile = fopen(input_file, "rb");

    if(infile == NULL)
    {
        perror("fopen");
        exit(EXIT_FAILURE);
    }


    /* Encode input frames */
    int numframes = 0;
    while(!feof(infile))
    {
        image = read_yuv(infile);

        if (!image) {
            break;
        }

        fprintf(stderr, "Encoding frame %d, ", numframes);
        c63_encode_image(cm, image);

        free(image->Y);
        free(image->U);
        free(image->V);
        free(image);

        fprintf(stderr, "Done!\n");

        ++numframes;
        if (limit_numframes && numframes >= limit_numframes)
            break;
    }

    fclose(outfile);
    fclose(infile);
//    fclose(predfile);
    
    mode = OFF_MODE;
    unlock();

    for (spe_nr = 0; spe_nr < NUM_SPE; spe_nr++)
        pthread_join(smart_thread[spe_nr], NULL);

    if (pthread_mutex_destroy (&mutex) != 0)
        perror("mutex destroy failed");

    spe_dispose();
    
    print_time();
    return EXIT_SUCCESS;
}
