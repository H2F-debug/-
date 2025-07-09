#ifndef GPIO_H
#define GPIO_H

/**
 * @brief 导出指定编号的GPIO（将GPIO从内核空间暴露到用户空间）
 * @param gpio_num GPIO编号（如GPIO123）
 * @return 0表示成功，-1表示失败
 */
int export_gpio(int gpio_num);

/**
 * @brief 设置GPIO的方向（输入或输出）
 * @param gpio_num GPIO编号
 * @param direction 方向字符串，"in"表示输入，"out"表示输出
 * @return 0表示成功，-1表示失败
 */
int set_direction(int gpio_num, const char *direction);

/**
 * @brief 设置GPIO的输出值（仅对输出方向有效）
 * @param gpio_num GPIO编号
 * @param value 输出值，0表示低电平，1表示高电平
 * @return 0表示成功，-1表示失败
 */
int set_value(int gpio_num, int value);

/**
 * @brief 微秒级延时函数
 * @param microseconds 延时时间（单位：微秒，1秒=1000000微秒）
 */
void delay_us(unsigned int microseconds);

#endif  // GPIO_H
