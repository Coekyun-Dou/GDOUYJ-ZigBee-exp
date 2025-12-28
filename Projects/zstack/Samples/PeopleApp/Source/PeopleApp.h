
#ifndef PeopleApp_H
#define PeopleApp_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "ZComDef.h"

//初始化一些终端地址
#define PeopleApp_ENDPOINT           10

#define PeopleApp_PROFID             0x0F04
#define PeopleApp_DEVICEID           0x0001
#define PeopleApp_DEVICE_VERSION     0
#define PeopleApp_FLAGS              0

#define PeopleApp_MAX_CLUSTERS       1
#define PeopleApp_CLUSTERID          1

// Send Message Timeout
#define PeopleApp_SEND_MSG_TIMEOUT   2000     // Every 2 seconds

// Application Events (OSAL) - These are bit weighted definitions.
#define PeopleApp_SEND_MSG_EVT       0x0001
#define PeopleApp_SEND_MSG_PERIODIC_BODY_EVT       0x0002

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * FUNCTIONS
 */

/*
 * Task Initialization for the Generic Application
 */
extern void PeopleApp_Init( byte task_id );

/*
 * Task Event Processor for the Generic Application
 */
extern UINT16 PeopleApp_ProcessEvent( byte task_id, UINT16 events );

#ifdef __cplusplus
}
#endif

#endif /* PeopleApp_H */
