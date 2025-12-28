
#ifndef seg7RouterApp_H
#define seg7RouterApp_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */
#include "ZComDef.h"

/*********************************************************************
 * CONSTANTS
 */

// These constants are only for example and should be changed to the
// device's needs
#define seg7RouterApp_ENDPOINT           10  应用端点号，

#define seg7RouterApp_PROFID             0x0F04  // 应用配置文件ID（Profile ID），用于标识设备所属的应用配置文件
#define seg7RouterApp_DEVICEID           0x0001  / 设备ID（Device ID），用于区分同一配置文件下的不同设备
#define seg7RouterApp_DEVICE_VERSION     0
#define seg7RouterApp_FLAGS              0   设备标志位，预留用于扩展设备特性标识

//#define seg7RouterApp_MAX_CLUSTERS              4 
#define seg7RouterApp_body_CLUSTERID            1
#define seg7RouterApp_temperature_CLUSTERID     2
#define seg7RouterApp_CLUSTERID                 5
#define UART_INPUT_Control_SendSeg7_CLUSTERID   10  
  
  
// Send Message Timeout
#define seg7RouterApp_SEND_MSG_TIMEOUT   10000     // Every 5 seconds

// Application Events (OSAL) - These are bit weighted definitions.
#define seg7RouterApp_SEND_MSG_EVT       0x0001

extern void seg7RouterApp_Init( byte task_id );

extern UINT16 seg7RouterApp_ProcessEvent( byte task_id, UINT16 events );


#ifdef __cplusplus
}
#endif

#endif /* seg7RouterApp_H */
