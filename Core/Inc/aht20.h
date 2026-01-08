#ifndef __AHT20_H
#define __AHT20_H

#include "main.h" // 包含主头文件以获取 HAL 库和类型定义

// AHT20 的 I2C 设备地址 (7-bit)
#define AHT20_I2C_ADDR          0x38

// AHT20 命令定义
#define AHT20_CMD_INIT          0xE1 // 初始化/校准命令
#define AHT20_CMD_MEASURE       0xAC // 触发测量命令
#define AHT20_CMD_SOFT_RESET    0xBA // 软复位命令

// 函数返回状态
typedef enum {
    AHT20_OK = 0,
    AHT20_ERROR,
    AHT20_NOT_DETECTED,
    AHT20_CRC_ERROR
} AHT20_StatusTypeDef;

// 温湿度数据结构
typedef struct {
    float temperature; // 摄氏度
    float humidity;    // 百分比 %
} AHT20_DataTypeDef;

// 函数声明
AHT20_StatusTypeDef AHT20_Init(void);
AHT20_StatusTypeDef AHT20_TriggerMeasurement(void);
AHT20_StatusTypeDef AHT20_ReadData(AHT20_DataTypeDef *data);
AHT20_StatusTypeDef AHT20_SoftReset(void);

#endif /* __AHT20_H */