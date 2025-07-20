#ifndef _USBLOGGER_H_
#define _USBLOGGER_H_


#include <pthread.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <time.h>
#include "stdio.h"
#include "stdlib.h"
#include "stdint.h"
#include <errno.h>

#define USB_MOUNT_PATH "/media/usb0"
#define LOG_FILE_PATH  USB_MOUNT_PATH "/data_log.csv"
#define Status_t int


typedef struct {

    void (*log_handler)(FILE *logf, void *shm_struct);
    void *shm_ptr;
    FILE *logf;
    pthread_mutex_t lock;

}st_UsbLogger;



#endif