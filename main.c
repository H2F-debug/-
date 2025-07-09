#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <signal.h>
#include "gpio.h"
#include "pwm.h"
#include "uart.h"  

#define K210_UART "/dev/ttyS8"  // K210连接的串口
#define TRACE_MODE1 0x01        // 循迹模式1指令
#define TRACE_MODE2 0x02        // 循迹模式2指令
#define TRACE_STOP 0x00         // 停止循迹指令

int fd_serial;       // 主控制串口
int fd_k210;         // 与K210通信的串口
volatile sig_atomic_t stop = 0;

// 数据包定义
#define PACKET_HEADER1 0x55
#define PACKET_HEADER2 0xAA
#define PACKET_FOOTER 0x0A
#define MAX_PACKET_LEN 32

typedef struct {
    unsigned char header1;
    unsigned char header2;
    unsigned char len;
    unsigned char cmd;
    unsigned char data[MAX_PACKET_LEN - 4]; // 减去header1,header2,len,footer
    unsigned char footer;
} SerialPacket;

/*
 * @description : 串口发送函数
 */
int func_send_frame(int fd, const unsigned char *p_send_buff, const int count)
{
    int Result = write(fd, p_send_buff, count);
    if (Result == -1) {
        perror("write");
    }
    return Result;
}

/*
 * @description : 串口接收函数
 */
int func_receive_frame(int fd, unsigned char *p_receive_buff, const int count)
{
    int nread = 0;
    fd_set rd;
    int retval = 0;
    struct timeval timeout = {1, 0}; // 1秒超时
    
    FD_ZERO(&rd);
    FD_SET(fd, &rd);
    memset(p_receive_buff, 0x0, count);
    
    retval = select(fd + 1, &rd, NULL, NULL, &timeout);
    switch(retval) {
        case 0:
            nread = 0; // 超时
            break;
        case -1:
            perror("select");
            nread = -1;
            break;
        default:
            nread = read(fd, p_receive_buff, count); // 读串口
            if (nread == -1) {
                perror("read");
            }
            break;
    }
    return nread;
}

/*
 * @description : 解析数据包
 */
int parse_packet(unsigned char *buffer, int len, SerialPacket *packet)
{
    int i = 0;
    int data_start = 0;
    
    // 查找帧头
    for (i = 0; i < len - 1; i++) {
        if (buffer[i] == PACKET_HEADER1 && buffer[i+1] == PACKET_HEADER2) {
            data_start = i;
            break;
        }
    }
    
    // 没有找到完整帧头
    if (i >= len - 1) {
        return 0;
    }
    
    // 检查数据长度是否足够
    if (len - data_start < 4) { // 至少需要header1,header2,len,footer
        return 0;
    }
    
    // 获取数据长度
    unsigned char packet_len = buffer[data_start + 2];
    
    // 检查数据包长度是否合理
    if (packet_len > MAX_PACKET_LEN - 4 || packet_len < 1) {
        return 0;
    }
    
    // 检查是否接收到完整数据包
    if (len - data_start < 3 + packet_len + 1) { // header1+header2+len+data+footer
        return 0;
    }
    
    // 检查帧尾
    if (buffer[data_start + 3 + packet_len] != PACKET_FOOTER) {
        return 0;
    }
    
    // 填充数据包结构
    packet->header1 = buffer[data_start];
    packet->header2 = buffer[data_start + 1];
    packet->len = packet_len;
    packet->cmd = buffer[data_start + 3]; // 命令字节
    
    // 复制数据部分
    if (packet_len > 1) {
        memcpy(packet->data, &buffer[data_start + 4], packet_len - 1);
    }
    
    packet->footer = buffer[data_start + 3 + packet_len];
    
    return 1;
}

/*
 * @description : 初始化K210通信串口
 */
int init_k210_uart()
{
    int fd;
    struct termios newtio, oldtio;
    
    // 打开与K210通信的串口
    fd = open(K210_UART, O_RDWR | O_NOCTTY);
    if (fd < 0) {
        perror(K210_UART);
        printf("无法打开K210串口 %s \n", K210_UART);
        return -1;
    }
    
    // 保存旧的串口配置
    if (tcgetattr(fd, &oldtio) != 0) {
        perror("tcgetattr");
        close(fd);
        return -1;
    }
    
    // 配置新的串口参数
    bzero(&newtio, sizeof(newtio));
    newtio.c_cflag |= CLOCAL | CREAD;  // 打开接收标志和忽略控制线
    newtio.c_cflag &= ~CSIZE;          // 清除数据位设置
    newtio.c_cflag |= CS8;             // 设置数据位为8位
    newtio.c_cflag &= ~PARENB;         // 无校验位
    newtio.c_cflag &= ~CSTOPB;         // 设置停止位1
    
    cfsetispeed(&newtio, B115200);     // K210通常使用115200波特率
    cfsetospeed(&newtio, B115200);
    
    newtio.c_cc[VTIME] = 1;            // 超时控制
    newtio.c_cc[VMIN] = 0;             // 不等待字符
    
    // 清空输入输出缓冲区
    tcflush(fd, TCIOFLUSH);
    
    // 应用新设置
    if ((tcsetattr(fd, TCSANOW, &newtio)) != 0) {
        perror("tcsetattr");
        close(fd);
        return -1;
    }
    
    return fd;
}

/*
 * @description : 向K210发送循迹控制命令
 */
int k210_send_command(unsigned char cmd)
{
    if (fd_k210 < 0) {
        printf("K210串口未初始化\n");
        return -1;
    }
    
    unsigned char buff[5];
    buff[0] = PACKET_HEADER1;
    buff[1] = PACKET_HEADER2;
    buff[2] = 0x01;  // 数据长度
    buff[3] = cmd;   // 命令
    buff[4] = PACKET_FOOTER;
    
    return write(fd_k210, buff, 5);
}

/*
 * @description : K210循迹模式1实现
 */
void trace_mode1()
{
    printf("启动循迹模式1: 快速循迹模式\n");
    if (k210_send_command(TRACE_MODE1) <= 0) {
        perror("发送循迹模式1命令失败");
    }
}

/*
 * @description : K210循迹模式2实现
 */
void trace_mode2()
{
    printf("启动循迹模式2: 精准循迹模式\n");
    if (k210_send_command(TRACE_MODE2) <= 0) {
        perror("发送循迹模式2命令失败");
    }
}

/*
 * @description : 停止K210循迹
 */
void stop_tracing()
{
    printf("停止循迹\n");
    k210_send_command(TRACE_STOP);
}

/*
 * @description : 信号处理函数
 */
void signal_handler(int signum)
{
    if (signum == SIGINT) {
        printf("\n接收到中断信号，准备退出...\n");
        stop_tracing();  // 停止循迹
        stop = 1;
    }
}

/*
 * @description : 主函数
 */
int main()
{
    int result = 0;
    unsigned char receive_buff[256];
    unsigned int receive_num = 0;
    SerialPacket packet;

    // 初始化K210通信串口
    fd_k210 = init_k210_uart();
    if (fd_k210 < 0) {
        printf("K210串口初始化失败，程序退出\n");
        return 1;
    }

    // 注册信号处理函数
    signal(SIGINT, signal_handler);
    
    // 打开控制串口
    fd_serial = open("/dev/ttyS9", O_RDWR | O_NOCTTY);
    if (fd_serial < 0) {
        perror("/dev/ttyS9");
        printf("无法打开控制串口 %s \n", "/dev/ttyS9");
        close(fd_k210);
        exit(1);
    }
    
    // 配置控制串口
    struct termios newtio, oldtio;
    if (tcgetattr(fd_serial, &oldtio) != 0) {
        perror("tcgetattr");
        close(fd_serial);
        close(fd_k210);
        return -1;
    }
    
    bzero(&newtio, sizeof(newtio));
    newtio.c_cflag |= CLOCAL | CREAD;
    newtio.c_cflag &= ~CSIZE;
    newtio.c_cflag |= CS8;
    newtio.c_cflag &= ~PARENB;
    newtio.c_cflag &= ~CSTOPB;
    
    cfsetispeed(&newtio, B9600);
    cfsetospeed(&newtio, B9600);
    
    newtio.c_cc[VTIME] = 0;
    newtio.c_cc[VMIN] = 0;
    
    tcflush(fd_serial, TCIOFLUSH);
    
    if ((tcsetattr(fd_serial, TCSANOW, &newtio)) != 0) {
        perror("tcsetattr");
        close(fd_serial);
        close(fd_k210);
        return -1;
    }
    
    printf("系统初始化完成，等待命令...\n");
    printf("发送字符1: 启动循迹模式1\n");
    printf("发送字符2: 启动循迹模式2\n");
    printf("按Ctrl+C: 退出程序\n");
    
    while (!stop) {
        // 数据接收
        receive_num = func_receive_frame(fd_serial, receive_buff, sizeof(receive_buff));
        if (receive_num > 0) {
            printf("[接收原始数据 %d 字节] ", receive_num);
            // 打印接收到的原始数据（十六进制）
            for (int i = 0; i < receive_num; i++) {
                printf("0x%02X ", receive_buff[i]);
            }
            putchar('\n');
            
            // 解析数据包
            if (parse_packet(receive_buff, receive_num, &packet)) {
                printf("解析到有效数据包: cmd=0x%02X, len=%d\n", packet.cmd, packet.len);
                
                // 处理命令
                switch (packet.cmd) {
                    case 0x01: 
                        trace_mode1();
                        break;
                    case 0x02: 
                        trace_mode2();
                        break;
                    case 0x00:
                        stop_tracing();
                        break;
                    default:
                        printf("未知命令: 0x%02X\n", packet.cmd);
                        break;
                }
            }
            
            // 检测字符命令
            for (int i = 0; i < receive_num; i++) {
                if (receive_buff[i] == '1') {
                    printf("检测到字符 '1'，启动循迹模式1\n");
                    trace_mode1();
                    break;
                }
                else if (receive_buff[i] == '2') {
                    printf("检测到字符 '2'，启动循迹模式2\n");
                    trace_mode2();
                    break;
                }
                else if (receive_buff[i] == '0') {
                    printf("检测到字符 '0'，停止循迹\n");
                    stop_tracing();
                    break;
                }
            }
        }
        
        // 短暂延时，降低CPU占用
        usleep(10000);
    }
    
    // 恢复旧的串口配置
    tcsetattr(fd_serial, TCSANOW, &oldtio);
    
    // 关闭串口
    close(fd_serial);
    close(fd_k210);
    
    printf("程序已退出\n");
    return 0;
}