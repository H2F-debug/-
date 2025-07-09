#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>

// 根据实际使用的PWM通道调整
#define PWM_CHIP_ID         0
#define PWM_CHANNEL_ID      0  // 改为0以匹配路径中的pwm0

#define PWM_EXPORT_PATH     "/sys/class/pwm/pwmchip%d/export"
#define PWM_UNEXPORT_PATH   "/sys/class/pwm/pwmchip%d/unexport"
#define PWM_ENABLE_PATH     "/sys/class/pwm/pwmchip%d/pwm%d/enable"
#define PWM_PERIOD_PATH     "/sys/class/pwm/pwmchip%d/pwm%d/period"
#define PWM_DUTY_CYCLE_PATH "/sys/class/pwm/pwmchip%d/pwm%d/duty_cycle"

// 生成完整路径的辅助函数
static char* get_pwm_path(const char* format, int chip_id, int channel_id) {
    static char path[128];
    snprintf(path, sizeof(path), format, chip_id, channel_id);
    return path;
}

// 检查文件是否存在
static int file_exists(const char* path) {
    return (access(path, F_OK) == 0);
}

// 导出PWM通道
int export_pwm(void) {
    char path[128];
    snprintf(path, sizeof(path), PWM_EXPORT_PATH, PWM_CHIP_ID);
    
    int fd = open(path, O_WRONLY);
    if (fd < 0) {
        if (errno == EBUSY) {
            printf("PWM通道%d已导出\n", PWM_CHANNEL_ID);
            return 0;
        }
        perror("无法打开export文件");
        return -1;
    }

    char channel_str[10];
    snprintf(channel_str, sizeof(channel_str), "%d", PWM_CHANNEL_ID);
    
    if (write(fd, channel_str, strlen(channel_str)) != (ssize_t)strlen(channel_str)) {
        perror("导出PWM通道失败");
        close(fd);
        return -1;
    }

    if (close(fd) < 0) {
        perror("关闭export文件失败");
        return -1;
    }
    
    // 等待文件系统更新
    usleep(100000);  // 100ms延迟
    
    // 验证导出是否成功
    snprintf(path, sizeof(path), PWM_PERIOD_PATH, PWM_CHIP_ID, PWM_CHANNEL_ID);
    if (!file_exists(path)) {
        fprintf(stderr, "PWM通道%d导出后文件未创建\n", PWM_CHANNEL_ID);
        return -1;
    }
    
    return 0;
}

// 取消导出PWM通道
int unexport_pwm(void) {
    char path[128];
    snprintf(path, sizeof(path), PWM_UNEXPORT_PATH, PWM_CHIP_ID);
    
    int fd = open(path, O_WRONLY);
    if (fd < 0) {
        if (errno == ENOENT) {
            printf("PWM通道%d未导出\n", PWM_CHANNEL_ID);
            return 0;
        }
        perror("无法打开unexport文件");
        return -1;
    }

    char channel_str[10];
    snprintf(channel_str, sizeof(channel_str), "%d", PWM_CHANNEL_ID);
    
    if (write(fd, channel_str, strlen(channel_str)) != (ssize_t)strlen(channel_str)) {
        perror("取消导出PWM通道失败");
        close(fd);
        return -1;
    }

    if (close(fd) < 0) {
        perror("关闭unexport文件失败");
        return -1;
    }
    
    // 等待文件系统更新
    usleep(100000);  // 100ms延迟
    
    return 0;
}

// 设置PWM周期（单位：纳秒）
int set_pwm_period(unsigned int period_ns) {
    char* path = get_pwm_path(PWM_PERIOD_PATH, PWM_CHIP_ID, PWM_CHANNEL_ID);
    
    int fd = open(path, O_WRONLY);
    if (fd < 0) {
        perror("无法打开period文件");
        return -1;
    }

    char buffer[32];
    snprintf(buffer, sizeof(buffer), "%u", period_ns);

    if (write(fd, buffer, strlen(buffer)) != (ssize_t)strlen(buffer)) {
        perror("设置PWM周期失败");
        close(fd);
        return -1;
    }

    if (close(fd) < 0) {
        perror("关闭period文件失败");
        return -1;
    }
    
    return 0;
}

// 设置PWM占空比（单位：纳秒）
int set_pwm_duty_cycle(unsigned int duty_ns) {
    char* path = get_pwm_path(PWM_DUTY_CYCLE_PATH, PWM_CHIP_ID, PWM_CHANNEL_ID);
    
    int fd = open(path, O_WRONLY);
    if (fd < 0) {
        perror("无法打开duty_cycle文件");
        return -1;
    }

    char buffer[32];
    snprintf(buffer, sizeof(buffer), "%u", duty_ns);

    if (write(fd, buffer, strlen(buffer)) != (ssize_t)strlen(buffer)) {
        perror("设置PWM占空比失败");
        close(fd);
        return -1;
    }

    if (close(fd) < 0) {
        perror("关闭duty_cycle文件失败");
        return -1;
    }
    
    return 0;
}

// 启用或禁用PWM输出
int enable_pwm(int enable) {
    char* path = get_pwm_path(PWM_ENABLE_PATH, PWM_CHIP_ID, PWM_CHANNEL_ID);
    
    int fd = open(path, O_WRONLY);
    if (fd < 0) {
        perror("无法打开enable文件");
        return -1;
    }

    char value = enable ? '1' : '0';

    if (write(fd, &value, 1) != 1) {
        perror("设置PWM启用状态失败");
        close(fd);
        return -1;
    }

    if (close(fd) < 0) {
        perror("关闭enable文件失败");
        return -1;
    }
    
    return 0;
}
