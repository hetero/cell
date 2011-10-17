CC = gcc
CFLAGS = -O3 -Wall -g -D_XOPEN_SOURCE=600
LDFLAGS = -lm -lspe2 -pthread 

SPU_CC = spu-gcc
SPU_CFLAGS = -O3 -Wall -g
SPU_LDFLAGS = 

all: c63enc c63dec

c63enc: c63enc.o dsp.o tables.o io.o c63_write.o c63.h common.o me.o sad_spe.elf spe.h ppe.h 

c63dec: c63dec.o dsp.o tables.o io.o c63.h common.o me.o

sad_spe.elf: sad_spe.c
	$(SPU_CC) $^ $(SPU_CFLAGS) $(SPU_LDFLAGS) -o $@   

clean:
	rm -f *.o *.elf c63enc c63dec
