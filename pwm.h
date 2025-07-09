#ifndef __PWM_H__
#define __PWM_H__

/**
 * @file pwm.h
 * @brief PWM控制接口，用于操作Linux内核的PWM子系统
 * @author doubao
 * @date 2025-07-04
 */

/**
 * @brief 导出PWM通道，使其可被操作
 * @return 成功返回0，失败返回-1
 */
int export_pwm(void);

/**
 * @brief 取消导出PWM通道，释放资源
 * @return 成功返回0，失败返回-1
 */
int unexport_pwm(void);

/**
 * @brief 设置PWM周期
 * @param period_ns 周期值，单位为纳秒
 * @return 成功返回0，失败返回-1
 */
int set_pwm_period(unsigned int period_ns);

/**
 * @brief 设置PWM占空比
 * @param duty_ns 占空比值，单位为纳秒，必须小于等于周期值
 * @return 成功返回0，失败返回-1
 */
int set_pwm_duty_cycle(unsigned int duty_ns);

/**
 * @brief 启用或禁用PWM输出
 * @param enable 1表示启用，0表示禁用
 * @return 成功返回0，失败返回-1
 */
int enable_pwm(int enable);

#endif /* __PWM_H__ */
