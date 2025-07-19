

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

#define I2C_BUS "dev/i2c-1"


static const char *TAG = "bh1750";


Status_t bh1750_read(bh1750_i2c_dev_t* bh1750_dev, uint16_t *level)
{
    pthread_mutex_lock(&bh1750_dev->lock);

    if(read(bh1750_dev->dev,&bh1750_dev->read_buf,2) < 0)
    {
        pthread_mutex_unlock(&bh1750_dev->lock);
        return STATUS_ERROR;

    }

    pthread_mutex_unlock(&bh1750_dev->lock);

    return STATUS_OK;
}



Status_t send_command(bh1750_i2c_dev_t* bh1750_dev, uint8_t cmd)
{
  
    pthread_mutex_lock(&bh1750_dev->lock);

    if(write(bh1750_dev->dev,bh1750_dev->write_buf,1) < 0)
    {
        pthread_mutex_unlock(&bh1750_dev->lock);
        return STATUS_ERROR;
    }

    pthread_mutex_unlock(&bh1750_dev->lock);
    
    return STATUS_OK;
}

Status_t bh1750_setup(bh1750_i2c_dev_t *bh1750_dev, bh1750_mode_t mode, bh1750_resolution_t resolution)
{
    uint8_t opcode;

    if(!bh1750_dev)
    return -1;

    mode = BH1750_MODE_CONTINUOUS ? OPCODE_CONT : OPCODE_OT;

    opcode = mode;

    switch (resolution)
    {
        case BH1750_RES_LOW:  opcode |= OPCODE_LOW;   break;
        case BH1750_RES_HIGH: opcode |= OPCODE_HIGH;  break;
        default:              opcode |= OPCODE_HIGH2; break;
    }

    if(send_command(bh1750_dev->dev,opcode) < 0)
    return STATUS_ERROR;

    return STATUS_OK;

}

Status_t bh1750_set_measurement_time(bh1750_i2c_dev_t *bh1750_dev, uint8_t time)
{
    
    if(send_command(bh1750_dev->dev,OPCODE_MT_HI | (time >> 5)) != STATUS_OK)
    return STATUS_ERROR;
    if(send_command(bh1750_dev->dev,OPCODE_MT_LO | (time & 0x1f) < 0))
    return STATUS_ERROR;

    return STATUS_OK;

}



void bh1750_dev_open(bh1750_i2c_dev_t *bh1750_dev)
{
   bh1750_dev->dev = open(I2C_BUS,O_RDWR);
   ioctl(bh1750_dev->dev,I2C_SLAVE,BH1750_ADDR);
   pthread_mutex_init(&bh1750_dev->lock,NULL);
   bh1750_setup(&bh1750_dev->dev, BH1750_MODE_CONTINUOUS, BH1750_RES_HIGH);




}