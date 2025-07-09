#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <time.h>

#define GPIO_PATH "/sys/class/gpio"
#define BUFFER_SIZE 100

// 导出GPIO
int export_gpio(int gpio_num) {
    char path[BUFFER_SIZE];
    int fd;

    sprintf(path, "%s/export", GPIO_PATH);
    fd = open(path, O_WRONLY);
    if (fd < 0) {
        perror("无法导出GPIO");
        return -1;
    }
    dprintf(fd, "%d", gpio_num);
    close(fd);
    return 0;
}

// 设置GPIO方向
int set_direction(int gpio_num, const char *direction) {
    char path[BUFFER_SIZE];
    int fd;

    sprintf(path, "%s/gpio%d/direction", GPIO_PATH, gpio_num);
    fd = open(path, O_WRONLY);
    if (fd < 0) {
        perror("无法设置方向");
        return -1;
    }
    dprintf(fd, "%s", direction);
    close(fd);
    return 0;
}

// 设置GPIO值
int set_value(int gpio_num, int value) {
    char path[BUFFER_SIZE];
    int fd;

    sprintf(path, "%s/gpio%d/value", GPIO_PATH, gpio_num);
    fd = open(path, O_WRONLY);
    if (fd < 0) {
        perror("无法设置值");
        return -1;
    }
    dprintf(fd, "%d", value);
    close(fd);
    return 0;
}

// 延时函数（微秒级）
void delay_us(unsigned int microseconds) {
    struct timespec ts;
    ts.tv_sec = microseconds / 1000000;
    ts.tv_nsec = (microseconds % 1000000) * 1000;
    nanosleep(&ts, NULL);
}


