
#ifndef TemperatureApp_H  
#define TemperatureApp_H

#ifdef __cplusplus  
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES 包含依赖头文件
 */
#include "ZComDef.h"  // 引入ZigBee通信协议核心定义头文件，提供ZigBee相关数据类型、常量和函数声明

/*********************************************************************
 * CONSTANTS 应用常量定义（示例值，可根据设备实际需求修改）
 */
#define TemperatureApp_ENDPOINT           10  // 应用端点号，ZigBee设备中用于标识特定应用的端点标识
#define TemperatureApp_PROFID             0x0F04  // 应用配置文件ID（Profile ID），用于标识设备所属的应用配置文件
#define TemperatureApp_DEVICEID           0x0001  // 设备ID（Device ID），用于区分同一配置文件下的不同设备
#define TemperatureApp_DEVICE_VERSION     0  // 设备版本号，4位二进制表示，用于标识设备固件版本
#define TemperatureApp_FLAGS              0  // 设备标志位，预留用于扩展设备特性标识
#define TemperatureApp_MAX_CLUSTERS       1  // 应用支持的最大簇数量，簇是ZigBee通信中数据分类和交互的基本单元
#define TemperatureApp_CLUSTERID          2  // 簇ID（Cluster ID），标识应用中具体的数据交互类型

#define TemperatureApp_SEND_MSG_TIMEOUT   3500     // 消息发送超时时间，单位为毫秒，此处设置为3.5秒
#define TemperatureApp_SEND_MSG_EVT       0x0001  // 应用层事件标识（OSAL事件位），用于标识消息发送事件

extern void TemperatureApp_Init( byte task_id );  // task_id：OSAL分配给本应用的任务ID，用于任务间通信和定时器操作

/*
 * Task Event Processor for the Generic Application
 * 通用应用任务事件处理器函数，处理本应用的所有事件（定时器、消息、按键等）
 */
extern UINT16 TemperatureApp_ProcessEvent( byte task_id, UINT16 events );  // task_id：任务ID；events：事件位图，包含待处理的所有事件

/*********************************************************************
*********************************************************************/
#ifdef __cplusplus
}  // extern "C"结束
#endif

#endif /* TemperatureApp_H */  // 头文件结束标记
