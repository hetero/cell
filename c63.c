#include "c63.h"
int mode;
pthread_mutex_t mutex;

void lock() {
    if (pthread_mutex_lock(&mutex) != 0)
        perror("lock failed");
}

void unlock() {
    if (pthread_mutex_unlock(&mutex) != 0)
        perror("unlock failed");
}
