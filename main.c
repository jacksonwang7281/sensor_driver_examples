#include "./include/bh1750_shm.h"
#include "./include/UsbLogger.h"
#include "fcntl.h"
#include <sys/mman.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <time.h>
#include "./include/UsbLogger.h"
#include "errno.h"



#define I2C_BUS "/dev/i2c-1"
#define USB_MOUNT_PATH "/media/usb0"
#define LOG_FILE_PATH  USB_MOUNT_PATH "/data_log.csv"
#define BH1750_SHM_NAME   "/bh1750_shm"


typedef struct {
    bh1750_i2c_dev_t *dev;
    shm_ring_t *shm;
} bh1750_thread_arg_t;

bh1750_i2c_dev_t bh1750_dev;

st_UsbLogger usblogger;


void log_ring_handler(FILE *logf, void *shm_struct) {

    shm_ring_t *ring = (shm_ring_t *)shm_struct;

    while (ring->tail != ring->head) {
        bh1750_data_t *m = &ring->data[ring->tail];

        // 建議補錯：避免未初始化的 lux 出現 NaN
        if (m->lux >= 0.0f && m->lux < 100000.0f)  
            fprintf(logf, "%llu,%.2f\n", (unsigned long long)m->ts_ns, m->lux);

        ring->tail = (ring->tail + 1) % RING_SIZE;
    }

    fflush(logf);
    usleep(5000); 

}


int device_init(void)
{
    int i2c_port = open(I2C_BUS,O_RDWR);

    if(i2c_port < 0)
    {
        printf("fail to open i2c bus\n");
        return EIO;
    }

    if(ioctl(i2c_port,I2C_SLAVE,BH1750_ADDR) < 0)
    {
        printf("failed to set I2C_SLAVE address");
        return EIO;
    }
    
    if(bh1750_init_desc(&bh1750_dev,BH1750_ADDR,i2c_port) < 0)
    {
        printf("device setup failed\n");
        return EIO;
    };

    if(usblogger_init(&usblogger,USB_MOUNT_PATH,BH1750_SHM_NAME,LOG_FILE_PATH,sizeof(shm_ring_t),log_ring_handler) < 0)
    {
        printf("device setup failed\n");
        return EIO;
    }

    return 0;

}



void* bh1750_sensor_saveToUsbTask(void* arg)
{
   st_UsbLogger *usblogger = (st_UsbLogger*)arg;

   while(1)
   {
   
        if (usblogger->log_handler && usblogger->logf && usblogger->shm_ptr) {
            usblogger->log_handler(usblogger->logf, usblogger->shm_ptr);
        }

   };


}



void* bh1750_sensor_readTask(void *arg)
{
    bh1750_thread_arg_t *ctx = (bh1750_thread_arg_t*) arg;
    bh1750_i2c_dev_t *dev = ctx->dev;
    shm_ring_t *shm_ring = ctx->shm;

    uint16_t raw;
    float lux;

    while (1) {
        if (bh1750_read(dev, &raw, &lux) == 0) {
            struct timespec ts;
            clock_gettime(CLOCK_REALTIME, &ts);

            pthread_mutex_lock(&shm_ring->lock);

            size_t next_head = (shm_ring->head + 1) % RING_SIZE;

            if (next_head == shm_ring->tail) {
                /* buffer full：跳過或覆蓋；此處選擇覆蓋最舊資料 */
                shm_ring->tail = (shm_ring->tail + 1) % RING_SIZE;
            }            

            shm_ring->data[shm_ring->head].lux = lux;

            shm_ring->data[shm_ring->head].ts_ns = ts.tv_sec * 1e9 + ts.tv_nsec;

            shm_ring->head = next_head;

            pthread_mutex_unlock(&shm_ring->lock);

            printf("[BH1750] Raw: 0x%04x, Lux: %.2f lx\n", raw, lux);
        } else {
            printf("[BH1750] Read failed\n");
        }

        usleep(500000);  // sleep 500ms
    }

    return NULL;
}


int main(){

    
    pthread_t sensor_thread;
    pthread_t usblog_thread;
    bh1750_thread_arg_t thread_arg;

    int shfd = shm_open(BH1750_SHM_NAME, O_CREAT | O_RDWR, 0666);
    ftruncate(shfd, sizeof(shm_ring_t));
    shm_ring_t *shm = mmap(NULL, sizeof(shm_ring_t), PROT_READ | PROT_WRITE,
                              MAP_SHARED, shfd, 0);                                           
    memset(shm,0,sizeof(shm_ring_t));  


    pthread_mutexattr_t attr;
    pthread_mutexattr_init(&attr);
    pthread_mutexattr_setpshared(&attr, PTHREAD_PROCESS_SHARED);
    pthread_mutex_init(&shm->lock, &attr);

    if (device_init() != 0) {
        printf("Device init failed\n");
        return -1;
    }

    thread_arg.dev = &bh1750_dev;
    thread_arg.shm = shm;

    if (pthread_create(&sensor_thread, NULL, bh1750_sensor_readTask, &thread_arg) != 0) {
        perror("pthread_create failed");
        return -1;
    }  

    if (pthread_create(&usblog_thread, NULL, bh1750_sensor_saveToUsbTask, &usblogger) != 0) {
        perror("pthread_create failed");
        return -1;
    }  




    // 5. 主線程可做別的事或等待
    pthread_join(sensor_thread, NULL);
    pthread_join(usblog_thread, NULL);

    return 0;
     

}