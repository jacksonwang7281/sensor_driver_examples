#include "./include/bh1750_shm.h"
#include "fcntl.h"
#include <sys/mman.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <time.h>

bh1750_i2c_dev_t bh1750_dev;
#define I2C_BUS "dev/i2c-1"


int device_init(void)
{
    int i2c_port = open(I2C_BUS,O_RDWR);

    if(i2c_port < 0)
    {
        printf("fail to open i2c bus\n");
        return STATUS_ERROR;
    }

    if(ioctl(i2c_port,I2C_SLAVE,BH1750_ADDR) < 0)
    {
        printf("failed to set I2C_SLAVE address");
        return STATUS_ERROR;
    }
    
    if(bh1750_init_desc(&bh1750_dev,BH1750_ADDR,i2c_port) < 0)
    {
        printf("device setup failed\n");
        return STATUS_ERROR;
    };

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