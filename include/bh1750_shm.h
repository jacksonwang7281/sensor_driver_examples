#ifndef BH1750_SHM_H
#define BH1750_SHM_H

#include <stdint.h>
#include <pthread.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <time.h>
#include <errno.h>


#define BH1750_SHM_SIZE   4096   // 4 KB，足夠放結構與一點預留
#define RING_SIZE 1024
#define Status_t int

#define BH1750_ADDR  0x23 //!< I2C address when ADDR pin floating/low

#define BH1750_ADDR_LO 0x23 //!< I2C address when ADDR pin floating/low
#define BH1750_ADDR_HI 0x5c //!< I2C address when ADDR pin high


typedef enum
{
    BH1750_MODE_ONE_TIME = 0, //!< One time measurement
    BH1750_MODE_CONTINUOUS    //!< Continuous measurement
} bh1750_mode_t;

/**
 * Measurement resolution
 */
typedef enum
{
    BH1750_RES_LOW = 0,  //!< 4 lx resolution, measurement time is usually 16 ms
    BH1750_RES_HIGH,     //!< 1 lx resolution, measurement time is usually 120 ms
    BH1750_RES_HIGH2     //!< 0.5 lx resolution, measurement time is usually 120 ms
} bh1750_resolution_t;


typedef struct {

    int port;
    uint8_t addr;
    uint8_t *write_buf;
    uint8_t *read_buf;

    pthread_mutex_t lock;

} bh1750_i2c_dev_t;

typedef struct {
    
    uint64_t        ts_ns;
    float           lux;
} bh1750_data_t;

typedef struct {
    bh1750_data_t data[RING_SIZE];
    pthread_mutex_t lock;
    size_t head;
    size_t tail; // 新增，讓 consumer 用來追資料
} shm_ring_t;









Status_t bh1750_init_desc(bh1750_i2c_dev_t *dev, uint8_t addr, int port);
Status_t bh1750_setup(bh1750_i2c_dev_t *dev, bh1750_mode_t mode, bh1750_resolution_t resolution);
Status_t bh1750_read(bh1750_i2c_dev_t *dev, uint16_t *lux_raw, float *lux_value);
Status_t bh1750_close(bh1750_i2c_dev_t *dev);

#endif