#include "aht20.h"
#include "i2c.h" // 使用您已有的 I2C2 句柄
#include "string.h" // 用于 memset

extern I2C_HandleTypeDef hi2c2; // 声明外部 I2C2 句柄

/**
 * @brief 计算 AHT20 数据的 CRC8 校验值
 * @param data: 待校验的数据指针
 * @param len: 数据长度
 * @return CRC8 校验值
 */
static uint8_t AHT20_CRC8(const uint8_t *data, uint8_t len) {
    uint8_t crc = 0xFF;
    uint8_t i, j;
    for (j = 0; j < len; j++) {
        crc ^= data[j];
        for (i = 0; i < 8; i++) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ 0x31;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

/**
 * @brief 检查 AHT20 是否在线
 * @return 状态码
 */
static AHT20_StatusTypeDef AHT20_CheckDevice(void) {
    uint8_t dummy;
    HAL_StatusTypeDef ret = HAL_I2C_Master_Receive(&hi2c2, (AHT20_I2C_ADDR << 1), &dummy, 1, 100);
    if (ret == HAL_OK) {
        return AHT20_OK;
    } else {
        return AHT20_NOT_DETECTED;
    }
}

/**
 * @brief 向 AHT20 发送初始化/校准命令
 * @return 状态码
 */
static AHT20_StatusTypeDef AHT20_SendInitCommand(void) {
    uint8_t cmd[3] = {AHT20_CMD_INIT, 0x08, 0x00};
    HAL_StatusTypeDef ret = HAL_I2C_Master_Transmit(&hi2c2, (AHT20_I2C_ADDR << 1), cmd, 3, 100);
    if (ret != HAL_OK) {
        return AHT20_ERROR;
    }
    
    // 等待初始化完成 (典型值 10ms)
    HAL_Delay(20);
    return AHT20_OK;
}

/**
 * @brief AHT20 传感器初始化函数
 * @return 状态码
 */
AHT20_StatusTypeDef AHT20_Init(void) {
    AHT20_StatusTypeDef status;
    
    // 1. 检查设备是否存在
    status = AHT20_CheckDevice();
    if (status != AHT20_OK) {
        return status;
    }
    
    // 2. 发送初始化命令
    status = AHT20_SendInitCommand();
    if (status != AHT20_OK) {
        return status;
    }
    
    return AHT20_OK;
}

/**
 * @brief 触发一次温湿度测量
 * @return 状态码
 */
AHT20_StatusTypeDef AHT20_TriggerMeasurement(void) {
    uint8_t cmd[3] = {AHT20_CMD_MEASURE, 0x33, 0x00};
    HAL_StatusTypeDef ret = HAL_I2C_Master_Transmit(&hi2c2, (AHT20_I2C_ADDR << 1), cmd, 3, 100);
    if (ret != HAL_OK) {
        return AHT20_ERROR;
    }
    return AHT20_OK;
}

/**
 * @brief 读取温湿度数据
 * @param data: 指向存储数据的结构体指针
 * @return 状态码
 */
AHT20_StatusTypeDef AHT20_ReadData(AHT20_DataTypeDef *data) {
    uint8_t rx_data[7] = {0};
    uint32_t raw_humidity, raw_temperature;
    
    // 读取 7 字节数据
    HAL_StatusTypeDef ret = HAL_I2C_Master_Receive(&hi2c2, (AHT20_I2C_ADDR << 1), rx_data, 7, 100);
    if (ret != HAL_OK) {
        return AHT20_ERROR;
    }
    
    // 检查忙标志位 (Bit 7 of Byte 0)
    if (rx_data[0] & 0x80) {
        return AHT20_ERROR; // 测量未完成
    }
    
    // CRC 校验 (前6字节)
    uint8_t crc_calculated = AHT20_CRC8(rx_data, 6);
    if (crc_calculated != rx_data[6]) {
        return AHT20_CRC_ERROR;
    }
    
    // 组合原始数据
    raw_humidity = ((uint32_t)rx_data[1] << 12) | ((uint32_t)rx_data[2] << 4) | (rx_data[3] >> 4);
    raw_temperature = ((uint32_t)(rx_data[3] & 0x0F) << 16) | ((uint32_t)rx_data[4] << 8) | rx_data[5];
    
    // 转换为物理量
    data->humidity = (float)raw_humidity * 100.0f / 1048576.0f; // 2^20 = 1048576
    data->temperature = (float)raw_temperature * 200.0f / 1048576.0f - 50.0f;
    
    return AHT20_OK;
}

/**
 * @brief 软复位 AHT20
 * @return 状态码
 */
AHT20_StatusTypeDef AHT20_SoftReset(void) {
    uint8_t cmd = AHT20_CMD_SOFT_RESET;
    HAL_StatusTypeDef ret = HAL_I2C_Master_Transmit(&hi2c2, (AHT20_I2C_ADDR << 1), &cmd, 1, 100);
    if (ret != HAL_OK) {
        return AHT20_ERROR;
    }
    HAL_Delay(20); // 复位后需要等待
    return AHT20_OK;
}