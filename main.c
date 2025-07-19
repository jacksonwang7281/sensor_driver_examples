#include "./include/bh1750_shm.h"
#include "fcntl.h"
#include <sys/mman.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <time.h>

bh1750_i2c_dev_t bh1750_dev;


void device_init(void)
{
    bh1750_dev_open(&bh1750_dev);



}


int main(){

    int shfd = shm_open(BH1750_SHM_NAME, O_CREAT | O_RDWR, 0666);
    ftruncate(shfd, BH1750_SHM_SIZE);
    bh1750_data_t *shm = mmap(NULL, BH1750_SHM_SIZE, PROT_READ | PROT_WRITE,
                              MAP_SHARED, shfd, 0);    


    pthread_mutexattr_t attr;
    pthread_mutexattr_init(&attr);
    pthread_mutexattr_setpshared(&attr, PTHREAD_PROCESS_SHARED);
    pthread_mutex_init(&shm->lock, &attr);

    device_init();

     

}