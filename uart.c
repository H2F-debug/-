#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include "uart.h"

/* 将波特率转换为系统定义的常量 */
static speed_t baud_to_speed(uint32_t baud) {
    switch (baud) {
        case UART_BAUD_9600:    return B9600;
        case UART_BAUD_19200:   return B19200;
        case UART_BAUD_38400:   return B38400;
        case UART_BAUD_57600:   return B57600;
        case UART_BAUD_115200:  return B115200;
        case UART_BAUD_230400:  return B230400;
        case UART_BAUD_460800:  return B460800;
        case UART_BAUD_921600:  return B921600;
        default:                return -1;
    }
}

/* 初始化串口通信 */
int uart_init(const char *dev, uint32_t baud, uint8_t data_bits, uint8_t parity, uint8_t stop_bits) {
    int fd;
    struct termios options;
    
    /* 打开串口设备 */
    fd = open(dev, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd < 0) {
        perror("uart_init: open failed");
        return UART_ERR_OPEN;
    }
    
    /* 配置串口参数 */
    if (tcgetattr(fd, &options) != 0) {
        perror("uart_init: tcgetattr failed");
        close(fd);
        return UART_ERR_CONFIG;
    }
    
    /* 设置波特率 */
    speed_t speed = baud_to_speed(baud);
    if (speed == -1) {
        fprintf(stderr, "uart_init: invalid baud rate: %u\n", baud);
        close(fd);
        return UART_ERR_PARAM;
    }
    
    if (cfsetispeed(&options, speed) != 0 || cfsetospeed(&options, speed) != 0) {
        perror("uart_init: cfsetispeed/cfsetospeed failed");
        close(fd);
        return UART_ERR_CONFIG;
    }
    
    /* 设置数据位 */
    options.c_cflag &= ~CSIZE;
    switch (data_bits) {
        case UART_DATA_BITS_5: options.c_cflag |= CS5; break;
        case UART_DATA_BITS_6: options.c_cflag |= CS6; break;
        case UART_DATA_BITS_7: options.c_cflag |= CS7; break;
        case UART_DATA_BITS_8: options.c_cflag |= CS8; break;
        default:
            fprintf(stderr, "uart_init: invalid data bits: %u\n", data_bits);
            close(fd);
            return UART_ERR_PARAM;
    }
    
    /* 设置校验位 */
    switch (parity) {
        case UART_PARITY_NONE:
            options.c_cflag &= ~PARENB;
            break;
        case UART_PARITY_ODD:
            options.c_cflag |= (PARENB | PARODD);
            break;
        case UART_PARITY_EVEN:
            options.c_cflag |= PARENB;
            options.c_cflag &= ~PARODD;
            break;
        default:
            fprintf(stderr, "uart_init: invalid parity: %u\n", parity);
            close(fd);
            return UART_ERR_PARAM;
    }
    
    /* 设置停止位 */
    switch (stop_bits) {
        case UART_STOP_BITS_1:
            options.c_cflag &= ~CSTOPB;
            break;
        case UART_STOP_BITS_2:
            options.c_cflag |= CSTOPB;
            break;
        default:
            fprintf(stderr, "uart_init: invalid stop bits: %u\n", stop_bits);
            close(fd);
            return UART_ERR_PARAM;
    }
    
    /* 设置其他选项 */
    options.c_cflag |= (CLOCAL | CREAD);  /* 忽略调制解调器状态行，启用接收器 */
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);  /* 原始模式 */
    options.c_oflag &= ~OPOST;  /* 原始输出 */
    
    /* 设置超时参数（默认阻塞模式） */
    options.c_cc[VMIN] = 0;     /* 读取的最小字节数 */
    options.c_cc[VTIME] = 0;    /* 超时时间（以0.1秒为单位） */
    
    /* 应用配置 */
    if (tcsetattr(fd, TCSANOW, &options) != 0) {
        perror("uart_init: tcsetattr failed");
        close(fd);
        return UART_ERR_CONFIG;
    }
    
    /* 清空输入/输出缓冲区 */
    tcflush(fd, TCIOFLUSH);
    
    return fd;
}

/* 从串口读取数据 */
ssize_t uart_read(int fd, uint8_t *buf, size_t len, uint32_t timeout_ms) {
    if (fd < 0 || buf == NULL || len == 0) {
        errno = EINVAL;
        return UART_ERR_PARAM;
    }
    
    /* 如果需要设置超时 */
    if (timeout_ms > 0) {
        struct termios options;
        if (tcgetattr(fd, &options) != 0) {
            perror("uart_read: tcgetattr failed");
            return UART_ERR_CONFIG;
        }
        
        /* 设置超时（以0.1秒为单位） */
        options.c_cc[VMIN] = 0;
        options.c_cc[VTIME] = timeout_ms / 100;  /* 转换为0.1秒 */
        
        if (tcsetattr(fd, TCSANOW, &options) != 0) {
            perror("uart_read: tcsetattr failed");
            return UART_ERR_CONFIG;
        }
    }
    
    ssize_t bytes_read = read(fd, buf, len);
    if (bytes_read < 0) {
        perror("uart_read: read failed");
        return UART_ERR_IO;
    }
    
    return bytes_read;
}

/* 向串口发送数据 */
ssize_t uart_write(int fd, const uint8_t *buf, size_t len) {
    if (fd < 0 || buf == NULL || len == 0) {
        errno = EINVAL;
        return UART_ERR_PARAM;
    }
    
    ssize_t bytes_written = write(fd, buf, len);
    if (bytes_written < 0) {
        perror("uart_write: write failed");
        return UART_ERR_IO;
    }
    
    return bytes_written;
}

/* 配置串口超时参数 */
int uart_set_timeout(int fd, uint32_t timeout_ms) {
    if (fd < 0) {
        errno = EINVAL;
        return UART_ERR_PARAM;
    }
    
    struct termios options;
    if (tcgetattr(fd, &options) != 0) {
        perror("uart_set_timeout: tcgetattr failed");
        return UART_ERR_CONFIG;
    }
    
    /* 设置超时（以0.1秒为单位） */
    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = timeout_ms / 100;  /* 转换为0.1秒 */
    
    if (tcsetattr(fd, TCSANOW, &options) != 0) {
        perror("uart_set_timeout: tcsetattr failed");
        return UART_ERR_CONFIG;
    }
    
    return UART_SUCCESS;
}

/* 刷新串口缓冲区 */
int uart_flush(int fd, uint8_t flush_rx, uint8_t flush_tx) {
    if (fd < 0) {
        errno = EINVAL;
        return UART_ERR_PARAM;
    }
    
    int queue_selector = 0;
    if (flush_rx && flush_tx) {
        queue_selector = TCIOFLUSH;
    } else if (flush_rx) {
        queue_selector = TCIFLUSH;
    } else if (flush_tx) {
        queue_selector = TCOFLUSH;
    } else {
        return UART_SUCCESS;  /* 无需刷新 */
    }
    
    if (tcflush(fd, queue_selector) != 0) {
        perror("uart_flush: tcflush failed");
        return UART_ERR_IO;
    }
    
    return UART_SUCCESS;
}

/* 关闭串口通信 */
void uart_close(int fd) {
    if (fd >= 0) {
        close(fd);
    }
}