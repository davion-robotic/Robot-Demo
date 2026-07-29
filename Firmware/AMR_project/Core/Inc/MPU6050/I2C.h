#ifndef __I2C_H_
#define __I2C_H_

// 1. CHỈ BAO HÀM FILE HAL TỔNG VÀ FILE CẤU HÌNH MAIN TẠI ĐÂY
#include "stm32f4xx_hal.h"
#include "main.h"
#include "stdint.h"
#include <math.h>

#define STM32_HAL
#define MPU6050

#ifdef STM32_HAL
#define UNUSED(x) ((void)(x))
#define delay_ms  HAL_Delay
#define fabs      fabsf
#define min(a,b)  ((a<b)?a:b)

// Khai báo trước cấu trúc để tránh lỗi chưa định nghĩa khi compiler quét qua
struct int_param_s; 
static inline int reg_int_cb(struct int_param_s *int_param)
{
    UNUSED(int_param);
    return 0;
}

#define get_ms(timestamp) (*(timestamp) = HAL_GetTick(), 0)
#define __no_operation()  (0)
#endif

#define hi2cMPU6050 hi2c1

#define log_i(...)     do {} while (0)
#define log_e(...)     do {} while (0)

// 2. NGUYÊN MẪU HÀM GIAO TIẾP I2C CHUẨN CỦA THƯ VIỆN
HAL_StatusTypeDef i2c_write(uint8_t slave_addr, uint8_t reg_addr, uint8_t length, uint8_t const *data);
HAL_StatusTypeDef i2c_read(uint8_t slave_addr, uint8_t reg_addr, uint8_t length, uint8_t *data);
HAL_StatusTypeDef IICwriteBit(uint8_t slave_addr, uint8_t reg_addr, uint8_t bitNum, uint8_t data);
HAL_StatusTypeDef IICwriteBits(uint8_t slave_addr, uint8_t reg_addr, uint8_t bitStart, uint8_t length, uint8_t data);

#endif /* __I2C_H_ */