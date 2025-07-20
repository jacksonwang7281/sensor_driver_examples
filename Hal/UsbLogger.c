#include "../include/UsbLogger.h"



Status_t usblogger_init(st_UsbLogger* usb_log_dev,
                   const char *usb_mount_path,
                   const char *shm_name,
                   const char *log_filename,
                   size_t shm_size,
                   void (*log_handler)(FILE *logf, void *shm_struct)) 
{
    if(!usb_log_dev)
    return EINVAL;

    if (access(usb_mount_path, F_OK) != 0) {
        fprintf(stderr, "USB mount point %s not found.\n", usb_mount_path);
        return EIO;
    }

    char full_log_path[256];
    snprintf(full_log_path, sizeof(full_log_path), "%s/%s", usb_mount_path, log_filename);

    FILE *logf = fopen(full_log_path, "a");
    if (!logf) {
        perror("fopen log file");
        return EIO;
    }

    int fd = shm_open(shm_name, O_RDWR, 0666);
    if (fd < 0) {
        perror("shm_open");
        fclose(logf);
        return EIO;
    }

    void *shm_ptr = mmap(NULL, shm_size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    if (shm_ptr == MAP_FAILED) {
        perror("mmap");
        close(fd);
        fclose(logf);
        return EIO;
    }

    if (pthread_mutex_init(&usb_log_dev->lock, NULL) != 0) {
        perror("Mutex init failed");
        return EIO;
    }    

    usb_log_dev->log_handler = log_handler;
    usb_log_dev->logf = logf;
    usb_log_dev->shm_ptr = shm_ptr;

    return 0;

}

void data_log_record(st_UsbLogger* usb_log_dev)
{
    if(!usb_log_dev->log_handler)
    return;

    pthread_mutex_lock(&usb_log_dev->lock);
    usb_log_dev->log_handler(usb_log_dev->logf,usb_log_dev->shm_ptr);
    pthread_mutex_unlock(&usb_log_dev->lock);
}
















