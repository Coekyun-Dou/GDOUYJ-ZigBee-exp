/******************************************************************************
  Filename:       dimmablelightApp.c
  Revised:        $Date: 2010-12-21 10:27:34 -0800 (Tue, 21 Dec 2010) $
  Revision:       $Revision: 24670 $

  Description:    Generic Application (no Profile).


  Copyright 2004-2010 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user
  who downloaded the software, his/her employer (which must be your employer)
  and Texas Instruments Incorporated (the "License").  You may not use this
  Software unless you agree to abide by the terms of the License. The License
  limits your use, and you acknowledge, that the Software may not be modified,
  copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio
  frequency transceiver, which is integrated into your product. Other than for
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
******************************************************************************/

/*********************************************************************
  This application isn't intended to do anything useful, it is
  intended to be a simple example of an application's structure.

  This application sends "Hello World" to another "Generic"
  application every 15 seconds.  The application will also
  receive "Hello World" packets.

  The "Hello World" messages are sent/received as MSG type message.

  This applications doesn't have a profile, so it handles everything
  directly - itself.

  Key control:
    SW1:
    SW2:  initiates end device binding
    SW3:
    SW4:  initiates a match description request
*********************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include "OSAL.h"
#include "AF.h"
#include "ZDApp.h"
#include "ZDObject.h"
#include "ZDProfile.h"

#include "dimmablelightApp.h"
#include "DebugTrace.h"

//包含调色灯初始化头文件
#include"Dimmablelight.h"


#if !defined( WIN32 )
  #include "OnBoard.h"
#endif

/* HAL */
#include "hal_lcd.h"
#include "hal_led.h"
#include "hal_key.h"
#include "hal_uart.h"


// 调光灯控制引脚定义（根据调光灯PDF文档）
#define DIMLIGHT_BLUE_PIN           P0_1      // 蓝色控制引脚
#define DIMLIGHT_RED_PIN            P0_6      // 红色控制引脚
#define DIMLIGHT_GREEN_PIN          P0_7      // 绿色控制引脚




/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

// This list should be filled with Application specific Cluster IDs.
const cId_t dimmablelightApp_ClusterList[dimmablelightApp_MAX_CLUSTERS] =
{
  dimmablelightApp_CLUSTERID,//用于最基本的通信测试
  DIMLIGHT_BLUE_CLUSTERID,
  DIMLIGHT_RED_CLUSTERID,
  DIMLIGHT_GREEN_CLUSTERID,
  CentreApp_control_dimmablelight_CLUSTERID
};
//告网
const SimpleDescriptionFormat_t dimmablelightApp_SimpleDesc =
{
  dimmablelightApp_ENDPOINT,              //  int Endpoint;
  dimmablelightApp_PROFID,                //  uint16 AppProfId[2];
  dimmablelightApp_DEVICEID,              //  uint16 AppDeviceId[2];
  dimmablelightApp_DEVICE_VERSION,        //  int   AppDevVer:4;
  dimmablelightApp_FLAGS,                 //  int   AppFlags:4;
  dimmablelightApp_MAX_CLUSTERS,          //  byte  AppNumInClusters;
  (cId_t *)dimmablelightApp_ClusterList,  //  byte *pAppInClusterList;
  dimmablelightApp_MAX_CLUSTERS,          //  byte  AppNumInClusters;
  (cId_t *)dimmablelightApp_ClusterList   //  byte *pAppInClusterList;
};

// This is the Endpoint/Interface description.  It is defined here, but
// filled-in in dimmablelightApp_Init().  Another way to go would be to fill
// in the structure here and make it a "const" (in code space).  The
// way it's defined in this sample app it is define in RAM.
endPointDesc_t dimmablelightApp_epDesc;

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
byte dimmablelightApp_TaskID;   // Task ID for internal task/event processing
                          // This variable will be received when
                          // dimmablelightApp_Init() is called.
devStates_t dimmablelightApp_NwkState;


byte dimmablelightApp_TransID;  // This is the unique message ID (counter)

afAddrType_t dimmablelightApp_DstAddr;

/*********************************************************************
 * LOCAL FUNCTIONS
 */
void dimmablelightApp_ProcessZDOMsgs( zdoIncomingMsg_t *inMsg );//这是调色灯处理ZigBee网络管理消息的函数
void dimmablelightApp_HandleKeys( byte shift, byte keys );//调色灯的按键处理
void dimmablelightApp_MessageMSGCB( afIncomingMSGPacket_t *pckt );//将簇id转换为灯光信号
void dimmablelightApp_SendTheMessage( void );//调色灯的示例发送函数
static void DimLightRouterApp_ControlDimLight(uint8 colorType); //控制调色灯亮对应的颜色 
static void DimLightRouterApp_SendSuccessResponse(uint8 colorType); //回应协调器下达的指令 
void Delay5000ms(void);
static void Centreapp_control_dimmablelight_dispaly(afIncomingMSGPacket_t *pkt);//自定义
/*********************************************************************
 * NETWORK LAYER CALLBACKS
 */

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      dimmablelightApp_Init
 *
 * @brief   Initialization function for the Generic App Task.
 *          This is called during initialization and should contain
 *          any application specific initialization (ie. hardware
 *          initialization/setup, table initialization, power up
 *          notificaiton ... ).
 *
 * @param   task_id - the ID assigned by OSAL.  This ID should be
 *                    used to send messages and set timers.
 *
 * @return  none
 */
void dimmablelightApp_Init( byte task_id )
{
  dimmablelightApp_TaskID = task_id;
  dimmablelightApp_NwkState = DEV_INIT;
  dimmablelightApp_TransID = 0;

  
  DimLightRouterApp_InitDimLightHardware();  // 初始化调光灯硬件
    // 调色灯初始化为低电平（灯灭）
  DIMLIGHT_BLUE_PIN = 0;
  DIMLIGHT_RED_PIN = 0;
  DIMLIGHT_GREEN_PIN = 0;
  
  DimLightRouterApp_ControlDimLight(0);// 初始化所有灯为关闭状态
  
  
  // Device hardware initialization can be added here or in main() (Zmain.c).
  // If the hardware is application specific - add it here.
  // If the hardware is other parts of the device add it in main().

  dimmablelightApp_DstAddr.addrMode = (afAddrMode_t)AddrNotPresent;
  dimmablelightApp_DstAddr.endPoint = 0;
  dimmablelightApp_DstAddr.addr.shortAddr = 0;

  // Fill out the endpoint description.
  dimmablelightApp_epDesc.endPoint = dimmablelightApp_ENDPOINT;
  dimmablelightApp_epDesc.task_id = &dimmablelightApp_TaskID;
  dimmablelightApp_epDesc.simpleDesc
            = (SimpleDescriptionFormat_t *)&dimmablelightApp_SimpleDesc;
  dimmablelightApp_epDesc.latencyReq = noLatencyReqs;

  // Register the endpoint description with the AF
  afRegister( &dimmablelightApp_epDesc );

  // Register for all key events - This app will handle all key events
  RegisterForKeys( dimmablelightApp_TaskID );

  // Update the display
#if defined ( LCD_SUPPORTED )
    HalLcdWriteString( "dimmablelightApp", HAL_LCD_LINE_1 );
#endif

  ZDO_RegisterForZDOMsg( dimmablelightApp_TaskID, End_Device_Bind_rsp );
  ZDO_RegisterForZDOMsg( dimmablelightApp_TaskID, Match_Desc_rsp );
}

/*********************************************************************
 * @fn      dimmablelightApp_ProcessEvent
 *
 * @brief   Generic Application Task event processor.  This function
 *          is called to process all events for the task.  Events
 *          include timers, messages and any other user defined events.
 *
 * @param   task_id  - The OSAL assigned task ID.
 * @param   events - events to process.  This is a bit map and can
 *                   contain more than one event.
 *
 * @return  none
 */
UINT16 dimmablelightApp_ProcessEvent( byte task_id, UINT16 events )
{
  afIncomingMSGPacket_t *MSGpkt;//接收到的应用层消息
  afDataConfirm_t *afDataConfirm;//确认数据送达

  // Data Confirmation message fields
  byte sentEP;
  ZStatus_t sentStatus;
  byte sentTransID;       // This should match the value sent
  (void)task_id;  // Intentionally unreferenced parameter

  if ( events & SYS_EVENT_MSG )//有系统消息需要处理
  {
    MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( dimmablelightApp_TaskID );
    while ( MSGpkt )
    {
      switch ( MSGpkt->hdr.event )//事件类型字段
        
        
        /*KEY_CHANGE(0x01): 按键变化消息
?
AF_INCOMING_MSG_CMD(0x45): 接收到其他设备发来的数据
?
AF_DATA_CONFIRM_CMD(0x46): 数据发送确认（成功/失败）
?
ZDO_STATE_CHANGE(0x00): 网络状态变化（如入网成功）
?
ZDO_CB_MSG(0x00): ZDO层回调消息（绑定/服务发现响应）
        */
      {
        case ZDO_CB_MSG:
          dimmablelightApp_ProcessZDOMsgs( (zdoIncomingMsg_t *)MSGpkt );
          break;

        case KEY_CHANGE:
          dimmablelightApp_HandleKeys( ((keyChange_t *)MSGpkt)->state, ((keyChange_t *)MSGpkt)->keys );
          break;

        case AF_DATA_CONFIRM_CMD:
          // This message is received as a confirmation of a data packet sent.
          // The status is of ZStatus_t type [defined in ZComDef.h]
          // The message fields are defined in AF.h
          afDataConfirm = (afDataConfirm_t *)MSGpkt;
          sentEP = afDataConfirm->endpoint;
          sentStatus = afDataConfirm->hdr.status;
          sentTransID = afDataConfirm->transID;
          (void)sentEP;
          (void)sentTransID;

          // Action taken when confirmation is received.
          if ( sentStatus != ZSuccess )
          {
            // The data wasn't delivered -- Do something
          }
          break;

        case AF_INCOMING_MSG_CMD:
          dimmablelightApp_MessageMSGCB( MSGpkt );
          break;

        case ZDO_STATE_CHANGE:
          dimmablelightApp_NwkState = (devStates_t)(MSGpkt->hdr.status);
          if ( (dimmablelightApp_NwkState == DEV_ZB_COORD)
              || (dimmablelightApp_NwkState == DEV_ROUTER)
              || (dimmablelightApp_NwkState == DEV_END_DEVICE) )
          {
            
            HalLedBlink(HAL_LED_2, 5, 50, 250);//成功加入网络闪烁link灯
            // Start sending "the" message in a regular interval.
              /*osal_start_timerEx( dimmablelightApp_TaskID,
                                dimmablelightApp_SEND_MSG_EVT,
                                dimmablelightApp_SEND_MSG_TIMEOUT );*/
          }
          break;

        default:
          break;
      }

      // Release the memory
      osal_msg_deallocate( (uint8 *)MSGpkt );

      // Next
      MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( dimmablelightApp_TaskID );
    }

    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }

  // Send a message out - This event is generated by a timer
  //  (setup in dimmablelightApp_Init()).
  if ( events & dimmablelightApp_SEND_MSG_EVT )
  {
    // Send "the" message
    dimmablelightApp_SendTheMessage();

    // Setup to send message again
    osal_start_timerEx( dimmablelightApp_TaskID,
                        dimmablelightApp_SEND_MSG_EVT,
                        dimmablelightApp_SEND_MSG_TIMEOUT );

    // return unprocessed events
    return (events ^ dimmablelightApp_SEND_MSG_EVT);
  }

  // Discard unknown events
  return 0;
}

/*********************************************************************
 * Event Generation Functions
 */

/*********************************************************************
 * @fn      dimmablelightApp_ProcessZDOMsgs()
 *
 * @brief   Process response messages
 *
 * @param   none
 *
 * @return  none
 */
void dimmablelightApp_ProcessZDOMsgs( zdoIncomingMsg_t *inMsg )
{
  switch ( inMsg->clusterID )
  {
    case End_Device_Bind_rsp:
      if ( ZDO_ParseBindRsp( inMsg ) == ZSuccess )
      {
        // Light LED
        HalLedSet( HAL_LED_4, HAL_LED_MODE_ON );
      }
#if defined( BLINK_LEDS )
      else
      {
        // Flash LED to show failure
        HalLedSet ( HAL_LED_4, HAL_LED_MODE_FLASH );
      }
#endif
      break;

    case Match_Desc_rsp:
      {
        ZDO_ActiveEndpointRsp_t *pRsp = ZDO_ParseEPListRsp( inMsg );
        if ( pRsp )
        {
          if ( pRsp->status == ZSuccess && pRsp->cnt )
          {
            dimmablelightApp_DstAddr.addrMode = (afAddrMode_t)Addr16Bit;
            dimmablelightApp_DstAddr.addr.shortAddr = pRsp->nwkAddr;
            // Take the first endpoint, Can be changed to search through endpoints
            dimmablelightApp_DstAddr.endPoint = pRsp->epList[0];

            // Light LED
            HalLedSet( HAL_LED_4, HAL_LED_MODE_ON );
          }
          osal_mem_free( pRsp );
        }
      }
      break;
  }
}

/*********************************************************************
 * @fn      dimmablelightApp_HandleKeys
 *
 * @brief   Handles all key events for this device.
 *
 * @param   shift - true if in shift/alt.
 * @param   keys - bit field for key events. Valid entries:
 *                 HAL_KEY_SW_4
 *                 HAL_KEY_SW_3
 *                 HAL_KEY_SW_2
 *                 HAL_KEY_SW_1
 *
 * @return  none
 */
void dimmablelightApp_HandleKeys( byte shift, byte keys )
{
  zAddrType_t dstAddr;

  // Shift is used to make each button/switch dual purpose.
  if ( shift )
  {
    if ( keys & HAL_KEY_SW_1 )
    {
    }
    if ( keys & HAL_KEY_SW_2 )
    {
    }
    if ( keys & HAL_KEY_SW_3 )
    {
    }
    if ( keys & HAL_KEY_SW_4 )
    {
    }
  }
  else
  {
    if ( keys & HAL_KEY_SW_1 )
    {
      // Since SW1 isn't used for anything else in this application...
#if defined( SWITCH1_BIND )
      // we can use SW1 to simulate SW2 for devices that only have one switch,
      keys |= HAL_KEY_SW_2;
#elif defined( SWITCH1_MATCH )
      // or use SW1 to simulate SW4 for devices that only have one switch
      keys |= HAL_KEY_SW_4;
#endif
    }

    if ( keys & HAL_KEY_SW_2 )
    {
      HalLedSet ( HAL_LED_4, HAL_LED_MODE_OFF );

      // Initiate an End Device Bind Request for the mandatory endpoint
      dstAddr.addrMode = Addr16Bit;
      dstAddr.addr.shortAddr = 0x0000; // Coordinator
      ZDP_EndDeviceBindReq( &dstAddr, NLME_GetShortAddr(),
                            dimmablelightApp_epDesc.endPoint,
                            dimmablelightApp_PROFID,
                            dimmablelightApp_MAX_CLUSTERS, (cId_t *)dimmablelightApp_ClusterList,
                            dimmablelightApp_MAX_CLUSTERS, (cId_t *)dimmablelightApp_ClusterList,
                            FALSE );
    }

    if ( keys & HAL_KEY_SW_3 )
    {
    }

    if ( keys & HAL_KEY_SW_4 )
    {
      HalLedSet ( HAL_LED_4, HAL_LED_MODE_OFF );
      // Initiate a Match Description Request (Service Discovery)
      dstAddr.addrMode = AddrBroadcast;
      dstAddr.addr.shortAddr = NWK_BROADCAST_SHORTADDR;
      ZDP_MatchDescReq( &dstAddr, NWK_BROADCAST_SHORTADDR,
                        dimmablelightApp_PROFID,
                        dimmablelightApp_MAX_CLUSTERS, (cId_t *)dimmablelightApp_ClusterList,
                        dimmablelightApp_MAX_CLUSTERS, (cId_t *)dimmablelightApp_ClusterList,
                        FALSE );
    }
  }
}

/*********************************************************************
 * LOCAL FUNCTIONS
 */

/*********************************************************************
 * @fn      dimmablelightApp_MessageMSGCB
 *
 * @brief   Data message processor callback.  This function processes
 *          any incoming data - probably from other devices.  So, based
 *          on cluster ID, perform the intended action.
 *
 * @param   none
 *
 * @return  none
 */
void dimmablelightApp_MessageMSGCB( afIncomingMSGPacket_t *pkt )
{
  switch (pkt->clusterId)
  {
    case DIMLIGHT_BLUE_CLUSTERID:   //收到来自于协调器的cid该cid表示的是人体感应 
      {  // 点亮蓝色灯
      DimLightRouterApp_ControlDimLight(1);  // 1=蓝色
      DimLightRouterApp_SendSuccessResponse(1);  // 发送蓝色成功响应，回显数据包给到协调器 
      break;
      }
      
    case DIMLIGHT_RED_CLUSTERID:   //收到来自于协调器的cid该cid表示的是温湿度 
      { // 点亮红色灯
      DimLightRouterApp_ControlDimLight(2);  // 2=红色
      DimLightRouterApp_SendSuccessResponse(2);  // 发送红色成功响应
      break;
      }
      
    case DIMLIGHT_GREEN_CLUSTERID:   //收到来自于协调器的cid该cid表示的是数码管回显 
      {// 点亮绿色灯
      DimLightRouterApp_ControlDimLight(3);  // 3=绿色
      DimLightRouterApp_SendSuccessResponse(3);  // 发送绿色成功响应
      break;
      }
      
    case CentreApp_control_dimmablelight_CLUSTERID:    //收到来自于协调器的自定义颜色的簇id
      {
          if( pkt->cmd.Data[7]==0xA5)
          {
            Centreapp_control_dimmablelight_dispaly(pkt);   //获取协调器要求的自定义颜色的数据，再显示混合的颜色
          }  
      } 
       
    default:
      // 未知Cluster ID
      break;
  }
}

/*********************************************************************
 * @fn      dimmablelightApp_SendTheMessage
 *
 * @brief   Send "the" message.
 *
 * @param   none
 *
 * @return  none
 */
void dimmablelightApp_SendTheMessage( void )
{
  char theMessageData[] = "Hello World";

  if ( AF_DataRequest( &dimmablelightApp_DstAddr, &dimmablelightApp_epDesc,
                       dimmablelightApp_CLUSTERID,
                       (byte)osal_strlen( theMessageData ) + 1,
                       (byte *)&theMessageData,
                       &dimmablelightApp_TransID,
                       AF_DISCV_ROUTE, AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
  {
    // Successfully requested to be sent.
  }
  else
  {
    // Error occurred in request to send.
  }
}




static void DimLightRouterApp_ControlDimLight(uint8 colorType)
{
  // 首先关闭所有灯
  P0_1 = 0;  // 蓝色灯DIMLIGHT_BLUE_PIN
  P0_6 = 0;  // 红色灯DIMLIGHT_RED_PIN
  P0_7 = 0;  // 绿色灯DIMLIGHT_GREEN_PIN
  
  // 根据颜色类型点亮对应的灯
  switch (colorType)
  {
    case 1:  // 蓝色
      { 
        LED_change(100,0 ,0 );
      Delay5000ms();
        break;
      }
      
    case 2:  // 红色
      { 
        LED_change(0,100,0 );
        Delay5000ms();
      
      break;
      } 
    case 3:  // 绿色
      { 
        LED_change(0,0,100);
        Delay5000ms();
      break;
      }
    default:
      // 未知颜色或0，保持所有灯关闭
      break;
  }
}


static void Centreapp_control_dimmablelight_dispaly(afIncomingMSGPacket_t *pkt)
{
              unsigned char red  =(unsigned char)pkt->cmd.Data[15];   //收到协调器发来的控制红色颜色的亮度
              unsigned char green =(unsigned char)pkt->cmd.Data[16];   //收到协调器发来的控制绿色颜色的亮度
              unsigned char blue =(unsigned char)pkt->cmd.Data[17];   //收到协调器发来的控制蓝色颜色的亮度                                
             LED_change((blue * 100) / 255,(red * 100) / 255,(green * 100) / 255);  //把调的亮度0-255映射为占空比0-100 
             DelayMilliseconds(5000);                
}


static void DimLightRouterApp_SendSuccessResponse(uint8 colorType)
{
  uint8 txFrame[32];
  uint8 i = 0;
  uint8 checkSum = 0;
  static uint8 responseSeq = 0;  // 响应序列号
  
  // 获取本节点地址和父节点地址
  uint16 myShortAddr = NLME_GetShortAddr();
  uint16 parentShortAddr = NLME_GetCoordShortAddr();
  
  // 获取节点类型（路由器）
  uint8 nodeType = 0x01;  // 路由节点
  
  // 根据协议文档4.2.2节和4.2.4节构建响应帧
  // 按照"上传传感器信息"格式，但包含地址信息（类似上传路由信息）
  
  // 1. 构建帧头
  txFrame[i++] = 0xAA;               // 帧头
  txFrame[i++] = 0x00;               // 帧长度（后面计算）
  txFrame[i++] = 0x02;               // 帧类型: 无线传感数据
  txFrame[i++] = responseSeq++;      // 帧序号
  
  // 2. 命令类型和协议类型
  txFrame[i++] = 0x03;               // 命令类型: 上传传感器信息，上传路由信息 
  txFrame[i++] = 0x01;               // 协议类型: Zigbee
  
  // 3. 传感器类型和索引
  txFrame[i++] =0x00;  // 调光灯传感器类型高字节
  txFrame[i++] =0xA5 ;  // 调光灯传感器类型低字节
  txFrame[i++] =0x00;               // 索引号
  
  // 4. 源地址（本节点地址）老样子采用的是大端模式，具体就是高位源地址放在数据帧低字节处 
  txFrame[i++] = HI_UINT16(myShortAddr);      // 源地址高字节
  txFrame[i++] = LO_UINT16(myShortAddr);      // 源地址低字节
  
  // 5. 父地址
  txFrame[i++] = HI_UINT16(parentShortAddr);  // 父地址高字节
  txFrame[i++] = LO_UINT16(parentShortAddr);  // 父地址低字节
  
  // 6. 节点类型
  txFrame[i++] = nodeType;           // 节点类型: 0x01=路由节点
  
  // 7. 值类型（根据协议，调光灯是特殊类型F0，3字节）
  txFrame[i++] = 0xF0;               // 值类型: 特殊类型
  
  // 8. 数据值（RGB三个亮度值）
  uint8 redValue = (colorType == 2) ? 0xFF : 0x00;   // 红色亮度
  uint8 greenValue = (colorType == 3) ? 0xFF : 0x00; // 绿色亮度
  uint8 blueValue = (colorType == 1) ? 0xFF : 0x00;  // 蓝色亮度
  
  txFrame[i++] = redValue;           // 红色亮度
  txFrame[i++] = greenValue;         // 绿色亮度
  txFrame[i++] = blueValue;          // 蓝色亮度
  
  // 9. 更新帧长度（包括帧长度字节本身）
  txFrame[1] = i + 1;                // 总字节数（包括校验码）
  
  // 10. 计算校验和
  for (uint8 j = 0; j < i; j++)
  {
    checkSum += txFrame[j];
  }
  txFrame[i++] = checkSum;
  
  
  
  // 8. 发送数据到协调器
  afAddrType_t dstAddr;
  dstAddr.addrMode = (afAddrMode_t)afAddr16Bit;
  dstAddr.endPoint = dimmablelightApp_ENDPOINT;//发送到对应设备（本次实训中是协调器）的对应端点10上 
  dstAddr.addr.shortAddr = 0x0000;   // 协调器地址
  
  if (AF_DataRequest(&dstAddr,//目标设备的地址
                     &dimmablelightApp_epDesc,//源设备的端点描述符号，在前面已经定义了 ，在前面已经注册了这个端口号10了。 可见发送到目的设备的端口号要跟自身设备的端口号相同！！！ 
                     dimmablelightApp_CLUSTERID,//发送本次信息所带的CID  
                     i,              // 数据长度
                     txFrame,
                     &dimmablelightApp_TransID,
                     AF_DISCV_ROUTE,
                     AF_DEFAULT_RADIUS) == afStatus_SUCCESS)
  {
    // 发送成功
    HalLedBlink(HAL_LED_2, 5, 250, 50);  // link闪烁表示发送成功
  } else
    {
       HalLedBlink(HAL_LED_1, 5, 250, 50);  //data闪烁表示发送失败 
    }
    
  }

void Delay5000ms(void) ////11.0592MHz
{
    unsigned char i, j, k;

    i = 43;
    j = 6;
    k = 203;
    do
    {
        do
        {
            while (--k);
        } while (--j);
    } while (--i);
}











