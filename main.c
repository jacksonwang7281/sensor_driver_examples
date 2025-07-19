#include "./include/bh1750_shm.h"
#include "fcntl.h"
#include <sys/mman.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <time.h>


#define I2C_BUS "dev/i2c-1"


typedef struct {
    bh1750_i2c_dev_t *dev;
    bh1750_data_t *shm;
} bh1750_thread_arg_t;


int device_init(bh1750_i2c_dev_t *bh1750_dev)
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

    return STATUS_OK;

}

void* bh1750_sensor_readTask(void *arg)
{
    bh1750_thread_arg_t *ctx = (bh1750_thread_arg_t*) arg;
    bh1750_i2c_dev_t *dev = ctx->dev;
    bh1750_data_t *shm = ctx->shm;

    uint16_t raw;
    float lux;

    while (1) {
        if (bh1750_read(dev, &raw, &lux) == STATUS_OK) {
            struct timespec ts;
            clock_gettime(CLOCK_REALTIME, &ts);

            pthread_mutex_lock(&shm->lock);
            shm->lux = lux;
            shm->ts_ns = ts.tv_sec * 1e9 + ts.tv_nsec;
            pthread_mutex_unlock(&shm->lock);

            printf("[BH1750] Raw: 0x%04x, Lux: %.2f lx\n", raw, lux);
        } else {
            printf("[BH1750] Read failed\n");
        }

        usleep(500000);  // sleep 500ms
    }

    return NULL;
}


int main(){

    bh1750_i2c_dev_t bh1750_dev;
    pthread_t sensor_thread;
    bh1750_thread_arg_t thread_arg;

    

    int shfd = shm_open(BH1750_SHM_NAME, O_CREAT | O_RDWR, 0666);
    ftruncate(shfd, BH1750_SHM_SIZE);
    bh1750_data_t *shm = mmap(NULL, BH1750_SHM_SIZE, PROT_READ | PROT_WRITE,
                              MAP_SHARED, shfd, 0);    


    pthread_mutexattr_t attr;
    pthread_mutexattr_init(&attr);
    pthread_mutexattr_setpshared(&attr, PTHREAD_PROCESS_SHARED);
    pthread_mutex_init(&shm->lock, &attr);

    if (device_init(&bh1750_dev) != STATUS_OK) {
        printf("Device init failed\n");
        return -1;
    }

    thread_arg.dev = &bh1750_dev;
    thread_arg.shm = shm;

    if (pthread_create(&sensor_thread, NULL, bh1750_sensor_readTask, &thread_arg) != 0) {
        perror("pthread_create failed");
        return -1;
    }  

    // 5. 主線程可做別的事或等待
    pthread_join(sensor_thread, NULL);

    return 0;
     

}