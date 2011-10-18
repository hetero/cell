CC = gcc
CFLAGS = -O3 -Wall -g -D_XOPEN_SOURCE=600 -maltivec
LDFLAGS = -lm 

all: c63enc c63dec

c63enc: c63enc.o dsp.o tables.o io.o c63_write.o common.o me.o
c63dec: c63dec.o dsp.o tables.o io.o common.o me.o

clean:
	rm -f *.o c63enc c63dec
