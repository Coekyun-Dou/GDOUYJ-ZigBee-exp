/**************************************************************************************************
  Filename:       dimmablelightApp.h
  Revised:        $Date: 2007-10-27 17:22:23 -0700 (Sat, 27 Oct 2007) $
  Revision:       $Revision: 15795 $

  Description:    This file contains the Generic Application definitions.


  Copyright 2004-2007 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user
  who downloaded the software, his/her employer (which must be your employer)
  and Texas Instruments Incorporated (the "License").  You may not use this
  Software unless you agree to abide by the terms of the License. The License
  limits your use, and you acknowledge, that the Software may not be modified,
  copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio
  frequency transceiver, which is integrated into your product.  Other than for
  the foregoing purpose, you may not use, reproduce, copy, prepare derivative
  works of, modify, distribute, perform, display or sell this Software and/or
  its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED 揂S IS?WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED, 
  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE, 
  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
  TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
  INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
  OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
  OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

  Should you have any questions regarding your right to use this Software,
  contact Texas Instruments Incorporated at www.TI.com. 
**************************************************************************************************/

#ifndef dimmablelightApp_H
#define dimmablelightApp_H

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
#define dimmablelightApp_ENDPOINT           10

#define dimmablelightApp_PROFID             0x0F04
#define dimmablelightApp_DEVICEID           0x0001
#define dimmablelightApp_DEVICE_VERSION     0
#define dimmablelightApp_FLAGS              0

#define dimmablelightApp_MAX_CLUSTERS       5
#define dimmablelightApp_CLUSTERID          6  //这个CID是调色灯节点回应协调器发送数据的时候用的CID 

// Send Message Timeout
#define dimmablelightApp_SEND_MSG_TIMEOUT   5000     // Every 5 seconds

#define delay_1s   10000  
  

// Application Events (OSAL) - These are bit weighted definitions.
#define dimmablelightApp_SEND_MSG_EVT       0x0001
#define DIMLIGHT_BLUE_CLUSTERID     1    // 蓝色灯控制，对应body传感器 
#define DIMLIGHT_RED_CLUSTERID      2    // 红色灯控制，对应温湿度传感器 
#define DIMLIGHT_GREEN_CLUSTERID    5   // 绿色灯控制，对应于数码管成功回显给协调器后协调器下命令让调色灯亮绿色 
#define CentreApp_control_dimmablelight_CLUSTERID    10  
  
  
/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * FUNCTIONS
 */

/*
 * Task Initialization for the Generic Application
 */
extern void dimmablelightApp_Init( byte task_id );

/*
 * Task Event Processor for the Generic Application
 */
extern UINT16 dimmablelightApp_ProcessEvent( byte task_id, UINT16 events );

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* dimmablelightApp_H */
