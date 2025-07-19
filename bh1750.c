

#include "./include/bh1750_shm.h"




#define OPCODE_HIGH  0x0
#define OPCODE_HIGH2 0x1
#define OPCODE_LOW   0x3

#define OPCODE_CONT 0x10
#define OPCODE_OT   0x20

#define OPCODE_POWER_DOWN 0x00
#define OPCODE_POWER_ON   0x01
#define OPCODE_MT_HI      0x40
#define OPCODE_MT_LO      0x60

#define I2C_FREQ_HZ 400000




static const char *TAG = "bh1750";


Status_t bh1750_read(bh1750_i2c_dev_t* bh1750_dev, uint16_t *lux_raw, float *lux_value) 
{
    if (!bh1750_dev || !lux_raw || !lux_value) return STATUS_ERROR;

    pthread_mutex_lock(&bh1750_dev->lock);

    if(read(bh1750_dev->port,bh1750_dev->read_buf,2) < 0)
    {
        pthread_mutex_unlock(&bh1750_dev->lock);
        return STATUS_ERROR;

    }

    *lux_raw = (bh1750_dev->read_buf[0] << 8) | bh1750_dev->read_buf[1];
    *lux_value = (float)(*lux_raw) / 1.2f;

    pthread_mutex_unlock(&bh1750_dev->lock);

    return STATUS_OK;
}



Status_t send_command(bh1750_i2c_dev_t* bh1750_dev, uint8_t cmd)
{
  
    pthread_mutex_lock(&bh1750_dev->lock);

    bh1750_dev->write_buf[0] = cmd;

    if(write(bh1750_dev->port,bh1750_dev->write_buf,1) < 0)
    {
        pthread_mutex_unlock(&bh1750_dev->lock);
        return STATUS_ERROR;
    }

    pthread_mutex_unlock(&bh1750_dev->lock);
    
    return STATUS_OK;
}


Status_t bh1750_power_down(bh1750_i2c_dev_t* dev)
{   
    if(!dev)
        return STATUS_ERROR;

    return send_command(dev,OPCODE_POWER_DOWN);

}

Status_t bh1750_power_on(bh1750_i2c_dev_t* dev)
{
    if(!dev)
        return STATUS_ERROR;

    return send_command(dev,OPCODE_POWER_ON);

};




Status_t bh1750_setup(bh1750_i2c_dev_t *bh1750_dev, bh1750_mode_t mode, bh1750_resolution_t resolution)
{
    uint8_t opcode;

    if(!bh1750_dev)
    return STATUS_ERROR;

    opcode = mode == BH1750_MODE_CONTINUOUS ? OPCODE_CONT : OPCODE_OT;

    switch (resolution)
    {
        case BH1750_RES_LOW:  opcode |= OPCODE_LOW;   break;
        case BH1750_RES_HIGH: opcode |= OPCODE_HIGH;  break;
        default:              opcode |= OPCODE_HIGH2; break;
    }
  
    if(send_command(bh1750_dev,opcode) < 0)
    return STATUS_ERROR;

    return STATUS_OK;

}

Status_t bh1750_set_measurement_time(bh1750_i2c_dev_t *bh1750_dev, uint8_t time)
{
    
    if(send_command(bh1750_dev,OPCODE_MT_HI | (time >> 5)) != STATUS_OK)
    return STATUS_ERROR;
    if(send_command(bh1750_dev,OPCODE_MT_LO | (time & 0x1f) < 0))
    return STATUS_ERROR;

    return STATUS_OK;

}



Status_t bh1750_init_desc(bh1750_i2c_dev_t *bh1750_dev, uint8_t addr, int port)
{

    if (addr != BH1750_ADDR_LO && addr != BH1750_ADDR_HI)
    {
        printf("invalid I2C address");
        return STATUS_ERROR;
    }

    bh1750_dev->port = port;
    bh1750_dev->addr = addr;
    bh1750_dev->write_buf = (uint8_t*)malloc(sizeof(uint8_t)*2);
    bh1750_dev->read_buf = (uint8_t*)malloc(sizeof(uint8_t)*2);

    if (!bh1750_dev->write_buf || !bh1750_dev->read_buf) {
        perror("malloc failed");
        return STATUS_ERROR;
    }


    if (pthread_mutex_init(&bh1750_dev->lock, NULL) != 0) {
        perror("Mutex init failed");
        close(bh1750_dev->port);
        return STATUS_ERROR;
    }


    if (bh1750_setup(&bh1750_dev->port, BH1750_MODE_CONTINUOUS, BH1750_RES_HIGH) != STATUS_OK) {
        printf("Failed to setup BH1750\n");
        close(bh1750_dev->port);
        pthread_mutex_destroy(&bh1750_dev->lock);
        return STATUS_ERROR;
    }

    return STATUS_OK;


}

Status_t bh1750_close(bh1750_i2c_dev_t *dev) {
    if (!dev) return;

    pthread_mutex_destroy(&dev->lock);
    if (dev->write_buf) free(dev->write_buf);
    if (dev->read_buf) free(dev->read_buf);
    if (dev->port >= 0) close(dev->port);
}