// File: ccitt_false_16.h
#ifndef _CCITT_FALSE_16_H_
#define _CCITT_FALSE_16_H_

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 计算 CRC16 校验值
 * 
 * @param data_ptr 指向数据块的指针
 * @param data_len 数据块的长度（字节数）
 * @param start_value CRC计算的起始值
 * @param is_first_call 是否为第一次调用
 *                      true: 第一次调用，使用初始值，忽略start_value
 *                      false: 后续调用，使用start_value作为起始值
 * @return uint16_t 计算得到的 CRC16 校验值
 */
uint16_t ccitt_false_16_calculate(const uint8_t* data_ptr, 
                            uint32_t data_len, 
                            uint16_t start_value, 
                            bool is_first_call);

#ifdef __cplusplus
}
#endif

#endif /* ccitt_false_16_H */