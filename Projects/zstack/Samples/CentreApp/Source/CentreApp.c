/******************************************************************************
  Filename:       CentreApp.c
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

#include "CentreApp.h"
#include "DebugTrace.h"

#if !defined( WIN32 )
  #include "OnBoard.h"
#endif

/* HAL */
#include "hal_lcd.h"
#include "hal_led.h"
#include "hal_key.h"
#include "hal_uart.h"


/*全局变量*/
uint16 RxLen;        //串口接收数据长度
uint8 UartDataBuf[128]; //串口数据缓存区指针，全局变量！！！ 

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
const cId_t CentreApp_ClusterList[CentreApp_MAX_CLUSTERS] =
{
     CentreApp_CLUSTERID,
    CentreApp_body_CLUSTERID,
    CentreApp_temperature_CLUSTERID,
    CentreApp_seg7RourerApp_CLUSTERID,
    CentreApp_dimmablelight_CLUSTERID
};

const SimpleDescriptionFormat_t CentreApp_SimpleDesc =
{
  CentreApp_ENDPOINT,              //  int Endpoint;
  CentreApp_PROFID,                //  uint16 AppProfId[2];
  CentreApp_DEVICEID,              //  uint16 AppDeviceId[2];
  CentreApp_DEVICE_VERSION,        //  int   AppDevVer:4;
  CentreApp_FLAGS,                 //  int   AppFlags:4;
  CentreApp_MAX_CLUSTERS,          //  byte  AppNumInClusters;
  (cId_t *)CentreApp_ClusterList,  //  byte *pAppInClusterList;
  CentreApp_MAX_CLUSTERS,          //  byte  AppNumInClusters;
  (cId_t *)CentreApp_ClusterList   //  byte *pAppInClusterList;
};

// This is the Endpoint/Interface description.  It is defined here, but
// filled-in in CentreApp_Init().  Another way to go would be to fill
// in the structure here and make it a "const" (in code space).  The
// way it's defined in this sample app it is define in RAM.
endPointDesc_t CentreApp_epDesc;

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
byte CentreApp_TaskID;   // Task ID for internal task/event processing
                          // This variable will be received when
                          // CentreApp_Init() is called.
devStates_t CentreApp_NwkState;


byte CentreApp_TransID;  // This is the unique message ID (counter)

afAddrType_t CentreApp_DstAddr;

/*********************************************************************
 * LOCAL FUNCTIONS
 */
void CentreApp_ProcessZDOMsgs( zdoIncomingMsg_t *inMsg );
void CentreApp_HandleKeys( byte shift, byte keys );
void CentreApp_MessageMSGCB( afIncomingMSGPacket_t *pckt );
void CentreApp_SendTheMessage( void );
void  CentreApp_SendSeg7_DimLight_Control(uint16 cID,uint16 sensortype);//控制数码管亮数字和调色灯亮颜色滴！ 
static void CentreApp_rxCB(uint8 port, uint8 event);//串口回调函数啥也不需要做滴 
void  SEND_UART_INPUT_Control_SendSeg7_MESSAGE(uint8 *buf,uint16 len);
void  SEND_UART_INPUT_Control_Dimmablelight_MESSAGE(uint8 *buf,uint16 len);
/*********************************************************************
 * NETWORK LAYER CALLBACKS
 */

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      CentreApp_Init
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
void CentreApp_Init( byte task_id )
{
	
	  halUARTCfg_t uartConfig;//串口初始化 ，为了把协调器收到的所有节点的数据都打印出来形成拓扑结构 
      uartConfig.configured = TRUE;               //使能串口
      uartConfig.baudRate   = HAL_UART_BR_115200; //波特率115200
      uartConfig.flowControl = FALSE;             //关闭流控
      uartConfig.callBackFunc = CentreApp_rxCB;     //回调函数
       HalUARTOpen(HAL_UART_PORT_0, &uartConfig);  //打开串口

	
  CentreApp_TaskID = task_id;
  CentreApp_NwkState = DEV_INIT;
  CentreApp_TransID = 0;

  // Device hardware initialization can be added here or in main() (Zmain.c).
  // If the hardware is application specific - add it here.
  // If the hardware is other parts of the device add it in main().

  CentreApp_DstAddr.addrMode = (afAddrMode_t)AddrNotPresent;
  CentreApp_DstAddr.endPoint = 0;
  CentreApp_DstAddr.addr.shortAddr = 0;

  // Fill out the endpoint description.
  CentreApp_epDesc.endPoint = CentreApp_ENDPOINT;
  CentreApp_epDesc.task_id = &CentreApp_TaskID;
  CentreApp_epDesc.simpleDesc
            = (SimpleDescriptionFormat_t *)&CentreApp_SimpleDesc;
  CentreApp_epDesc.latencyReq = noLatencyReqs;

  // Register the endpoint description with the AF
  afRegister( &CentreApp_epDesc );

  // Register for all key events - This app will handle all key events
  RegisterForKeys( CentreApp_TaskID );

  // Update the display
#if defined ( LCD_SUPPORTED )
    HalLcdWriteString( "CentreApp", HAL_LCD_LINE_1 );
#endif

  ZDO_RegisterForZDOMsg( CentreApp_TaskID, End_Device_Bind_rsp );
  ZDO_RegisterForZDOMsg( CentreApp_TaskID, Match_Desc_rsp );
}

/*********************************************************************
 * @fn      CentreApp_ProcessEvent
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
UINT16 CentreApp_ProcessEvent( byte task_id, UINT16 events )
{
  afIncomingMSGPacket_t *MSGpkt;
  afDataConfirm_t *afDataConfirm;

  // Data Confirmation message fields
  byte sentEP;
  ZStatus_t sentStatus;
  byte sentTransID;       // This should match the value sent
  (void)task_id;  // Intentionally unreferenced parameter

  if ( events & SYS_EVENT_MSG )
  {
    MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( CentreApp_TaskID );  //接收数据包 
    while ( MSGpkt )
    {
      switch ( MSGpkt->hdr.event )
      {
        case ZDO_CB_MSG:
          CentreApp_ProcessZDOMsgs( (zdoIncomingMsg_t *)MSGpkt );
          break;

        case KEY_CHANGE:
          CentreApp_HandleKeys( ((keyChange_t *)MSGpkt)->state, ((keyChange_t *)MSGpkt)->keys );
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
          CentreApp_MessageMSGCB( MSGpkt );
          break;

        case ZDO_STATE_CHANGE:
          CentreApp_NwkState = (devStates_t)(MSGpkt->hdr.status);
          if ( (CentreApp_NwkState == DEV_ZB_COORD)
              || (CentreApp_NwkState == DEV_ROUTER)
              || (CentreApp_NwkState == DEV_END_DEVICE) )
          {
          	
          	HalLedBlink(HAL_LED_2, 5, 50, 250);//创建网络成功后闪烁link灯 
            // Start sending "the" message in a regular interval.
            /*osal_start_timerEx( CentreApp_TaskID,
                                CentreApp_SEND_MSG_EVT,
                                CentreApp_SEND_MSG_TIMEOUT );*/
          }
          break;

        default:
          break;
      }

      // Release the memory
      osal_msg_deallocate( (uint8 *)MSGpkt );

      // Next
      MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( CentreApp_TaskID );
    }

    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }

  // Send a message out - This event is generated by a timer
  //  (setup in CentreApp_Init()).
  if ( events & CentreApp_SEND_MSG_EVT )
  {
    // Send "the" message
    CentreApp_SendTheMessage();

    // Setup to send message again
    osal_start_timerEx( CentreApp_TaskID,
                        CentreApp_SEND_MSG_EVT,
                        CentreApp_SEND_MSG_TIMEOUT );

    // return unprocessed events
    return (events ^ CentreApp_SEND_MSG_EVT);
  }

  if ( events & CentreApp_UART_RX_CB_EVT )
  {
    /* uint16 RxLen;       
    uint8 UartDataBuf[128];   后面的触发串口输入回调事件已经定义这两个全局变量，里面存的是串口输入的数据*/ 

    if(UartDataBuf[7]==0xA2)   //如果串口输入的是数码管的数据
    {  
   SEND_UART_INPUT_Control_SendSeg7_MESSAGE(&UartDataBuf[0], RxLen);//取到串口输入数组的首地址和数组长度 &UartDataBuf[0]等价于UartDataBuf 
    }
     else  //如果串口输入的是调光灯的数据
   {
      SEND_UART_INPUT_Control_Dimmablelight_MESSAGE(&UartDataBuf[0], RxLen);//取到串口输入数组的首地址和数组长度 &UartDataBuf[0]等价于UartDataBuf 
   }
    // return unprocessed events
    return (events ^ CentreApp_UART_RX_CB_EVT);
  }


  // Discard unknown events
  return 0;
}

/*********************************************************************
 * Event Generation Functions
 */

/*********************************************************************
 * @fn      CentreApp_ProcessZDOMsgs()
 *
 * @brief   Process response messages
 *
 * @param   none
 *
 * @return  none
 */
void CentreApp_ProcessZDOMsgs( zdoIncomingMsg_t *inMsg )
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
            CentreApp_DstAddr.addrMode = (afAddrMode_t)Addr16Bit;
            CentreApp_DstAddr.addr.shortAddr = pRsp->nwkAddr;
            // Take the first endpoint, Can be changed to search through endpoints
            CentreApp_DstAddr.endPoint = pRsp->epList[0];

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
 * @fn      CentreApp_HandleKeys
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
void CentreApp_HandleKeys( byte shift, byte keys )
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
                            CentreApp_epDesc.endPoint,
                            CentreApp_PROFID,
                            CentreApp_MAX_CLUSTERS, (cId_t *)CentreApp_ClusterList,
                            CentreApp_MAX_CLUSTERS, (cId_t *)CentreApp_ClusterList,
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
                        CentreApp_PROFID,
                        CentreApp_MAX_CLUSTERS, (cId_t *)CentreApp_ClusterList,
                        CentreApp_MAX_CLUSTERS, (cId_t *)CentreApp_ClusterList,
                        FALSE );
    }
  }
}

/*********************************************************************
 * LOCAL FUNCTIONS
 */

/*********************************************************************
 * @fn      CentreApp_MessageMSGCB
 *
 * @brief   Data message processor callback.  This function processes
 *          any incoming data - probably from other devices.  So, based
 *          on cluster ID, perform the intended action.
 *
 * @param   none
 *
 * @return  none
 */
void CentreApp_MessageMSGCB( afIncomingMSGPacket_t *pkt )
{
  switch ( pkt->clusterId )
  {
    case CentreApp_CLUSTERID:  //这个cid没什么用的其实 
      // "the" message
      break;
      
    case CentreApp_body_CLUSTERID:  //接收到来自于人体感应发来的数据 	
    {
    uint16 body_sensor_type= (pkt->cmd.Data[6] << 8) | pkt->cmd.Data[7];   //我先从数据包里面获取传感器类型 
	CentreApp_SendSeg7_DimLight_Control(pkt->clusterId,body_sensor_type); 
	 HalUARTWrite(HAL_UART_PORT_0, &(pkt->cmd.Data[0]), pkt->cmd.DataLength ); //将收到的数据通过串口打印出来  	
	break;
	}
	case CentreApp_temperature_CLUSTERID:  //接收到来自于温湿度传感器发来的数据 
      {
      	uint16 temperature_sensor_type=(pkt->cmd.Data[6] << 8) | pkt->cmd.Data[7];  //数据包里面获取传感器类型 
       CentreApp_SendSeg7_DimLight_Control(pkt->clusterId,temperature_sensor_type); 
      /*  //还原温湿度值
        uint16 tempRaw = (pkt->cmd.Data[15] << 8) | pkt->cmd.Data[16];  // 温度原始值
        uint16 humiRaw = (pkt->cmd.Data[17] << 8) | pkt->cmd.Data[18];  // 湿度原始值
       // 4. 转换为实际值再放回数据帧里面去
        pkt->cmd.Data[15]=HI_UINT16((uint16)(tempRaw / 100.0f));
        pkt->cmd.Data[16]=LO_UINT16((uint16)(tempRaw / 100.0f));
        pkt->cmd.Data[17] =HI_UINT16((uint16)(humiRaw / 100.0f));
        pkt->cmd.Data[18] =LO_UINT16((uint16)(humiRaw / 100.0f));          不可以这样子先把数据还原了再去放回去，因为上位机会根据数据帧来判断该帧是否出错，如果我们把温湿度修改了再放回去就会出现上位机根据帧校验码认为该帧出现了问题 */        
         HalUARTWrite(HAL_UART_PORT_0, &(pkt->cmd.Data[0]), pkt->cmd.DataLength ); //将收到的数据通过串口打印出来 
      break;
      }
      
      
    case CentreApp_seg7RourerApp_CLUSTERID:    //接收到来自于数码管传感器发来的数据 
      {
      	uint16 seg7RourerApp_sensor_type=(pkt->cmd.Data[6] << 8) | pkt->cmd.Data[7];//数据包里面获取传感器类型 
      CentreApp_SendSeg7_DimLight_Control(pkt->clusterId,seg7RourerApp_sensor_type);
       HalUARTWrite(HAL_UART_PORT_0, &(pkt->cmd.Data[0]), pkt->cmd.DataLength ); //将收到的数据通过串口打印出来 
      break;
      }
      
      
      
     case CentreApp_dimmablelight_CLUSTERID:   //接收到来自于调色灯传感器发来的数据 
      HalUARTWrite(HAL_UART_PORT_0, &(pkt->cmd.Data[0]), pkt->cmd.DataLength ); //将收到的数据通过串口打印出来 
      break;
      
#if defined( LCD_SUPPORTED )
      HalLcdWriteScreen( (char*)pkt->cmd.Data, "rcvd" );
#elif defined( WIN32 )
      WPRINTSTR( pkt->cmd.Data );
#endif
      break;
  }
}

/*********************************************************************
 * @fn      CentreApp_SendTheMessage
 *
 * @brief   Send "the" message.
 *
 * @param   none
 *
 * @return  none
 */
void CentreApp_SendTheMessage( void )
{
  char theMessageData[] = "Hello World";

  if ( AF_DataRequest( &CentreApp_DstAddr, &CentreApp_epDesc,
                       CentreApp_CLUSTERID,
                       (byte)osal_strlen( theMessageData ) + 1,
                       (byte *)&theMessageData,
                       &CentreApp_TransID,
                       AF_DISCV_ROUTE, AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
  {
    // Successfully requested to be sent.
  }
  else
  {
    // Error occurred in request to send.
  }
}

/*********************************************************************
*********************************************************************/

void  CentreApp_SendSeg7_DimLight_Control(uint16 cID,uint16 sensortype)
{
	
	uint8 txFrame[32];
    uint8 i = 0;
    uint8 checkSum = 0;
    uint16 CLUSTERID =cID;//接收cid 
    uint16 sensor_type=sensortype;//接收对应的传感器类型 
    static uint8 responseSeq = 0; // 响应帧序号
  // 根据协议文档4.2.5节构建控制命令帧
  // 命令类型0x0F：下发控制执行器命令
  
  // 1. 构建帧头
  txFrame[i++] = 0xAA;               // 帧头
  txFrame[i++] = 0x00;               // 帧长度（后面计算）
  txFrame[i++] = 0x02;               // 帧类型: 无线传感数据
  txFrame[i++] = responseSeq++; // 帧序号
  
  // 2. 命令类型和协议类型
  txFrame[i++] = 0x0F;               // 命令类型: 下发控制执行器命令
  txFrame[i++] = 0x01;               // 协议类型: Zigbee
  
  // 3. 传感器类型和索引
  txFrame[i++] = HI_UINT16(sensor_type);  // 传感器类型高字节
  txFrame[i++] = LO_UINT16(sensor_type);  // 传感器类型低字节
  txFrame[i++] = 0x00;               // 索引号
  
  // 4. 源地址（协调器地址）
  uint16 myShortAddr = NLME_GetShortAddr();//获取协调器自身的地址！！！ 
  txFrame[i++] = HI_UINT16(myShortAddr);      // 源地址低字节
  txFrame[i++] = LO_UINT16(myShortAddr);      // 源地址高字节
  
  // 5. 父地址（协调器父地址为0xFFFF）协调器没有父节点，它自己就是根节点 
  txFrame[i++] = 0xFF;               // 父地址低字节
  txFrame[i++] = 0xFF;               // 父地址高字节
  
  // 6. 节点类型（协调器）
  txFrame[i++] = 0x00;               // 节点类型: 0x00=协调节点
  
  // 7. 值长度
  txFrame[i++] = 0x01;               // 值长度: 1字节
  
  // 8. 数据值（显示的值）
  txFrame[i++] = 0x00;       // 要显示的数据早已经通过CID区分开了，所以这里的数据咩野意义 
  
  // 9. 更新帧长度
  txFrame[1] = i + 1;                // 总字节数（包括校验码）
  
  // 10. 计算校验和
  for (uint8 j = 0; j < i; j++)
  {
    checkSum += txFrame[j];
  }
  txFrame[i++] = checkSum;
	
	afAddrType_t dstAddr;
  dstAddr.addrMode = (afAddrMode_t)AddrBroadcast;  //广播发送 
  dstAddr.endPoint = CentreApp_ENDPOINT;   //发送到目标设备的端点10上 
  dstAddr.addr.shortAddr = 0xFFFF;  //广播发送 
  
  if (AF_DataRequest(&dstAddr,
                     &CentreApp_epDesc,
                     CLUSTERID,       //发送函数接收到cID然后再发出去 
                     i,
                     txFrame,
                     &CentreApp_TransID,
                     AF_DISCV_ROUTE,
                     AF_DEFAULT_RADIUS) == afStatus_SUCCESS)
	{
		      HalLedBlink(HAL_LED_2, 5, 50, 250);//发送成功后闪烁link灯 
	}
  else
  {
  	          HalLedBlink(HAL_LED_1, 5, 50, 250);//发送失败后闪烁data灯 
  	
  }	
}



static void CentreApp_rxCB(uint8 port, uint8 event)  //串口回调函数 ，串口回调函数主要功能是获取用户从PC端串口输入的数据，并手动触发ChatApp_UART_RX _CB_EVT用户自定义事件。 
{
    if ((event & (HAL_UART_RX_FULL | HAL_UART_RX_ABOUT_FULL | HAL_UART_RX_TIMEOUT)))
  {
    RxLen = Hal_UART_RxBufLen(HAL_UART_PORT_0);  //接收缓冲区数据长度,字节为单位
    HalUARTRead( HAL_UART_PORT_0, &UartDataBuf[0], RxLen); //从串口0读接收缓冲区数据到内存
    osal_set_event(CentreApp_TaskID,CentreApp_UART_RX_CB_EVT); //有串口数据时触发相应事件
  }
                            //因为串口不需要接收什么数据 所以不需要写什么 
                      
}


void  SEND_UART_INPUT_Control_SendSeg7_MESSAGE(uint8 *buf,uint16 len) //串口接收数据自定义数字发给数码管
{
	
  afAddrType_t  SendSeg7_MESSAGE_dstAddr;
  SendSeg7_MESSAGE_dstAddr.addrMode = (afAddrMode_t)AddrBroadcast;  //广播发送 
  SendSeg7_MESSAGE_dstAddr.endPoint = CentreApp_ENDPOINT;   //发送到目标设备的端点10上 
  SendSeg7_MESSAGE_dstAddr.addr.shortAddr = 0xFFFF;  //广播发送 
	
  if( AF_DataRequest( &SendSeg7_MESSAGE_dstAddr, &CentreApp_epDesc,
                       CentreApp_CLUSTERID,
                       len,
                       buf,
                       &CentreApp_TransID,
                       AF_DISCV_ROUTE, AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
  {
    // 发送成功后闪烁 link灯 
     HalLedBlink( HAL_LED_2, 5, 50, 250 );
     osal_memset(buf, 0, 128);  //清除缓冲区
  }
  else
  {
    // Error occurred in request to send.
  }

}

void  SEND_UART_INPUT_Control_Dimmablelight_MESSAGE(uint8 *buf,uint16 len)  //串口接收数据自定义命令发给调色灯
{
  afAddrType_t  SendSeg7_MESSAGE_dstAddr;
  SendSeg7_MESSAGE_dstAddr.addrMode = (afAddrMode_t)AddrBroadcast;  //广播发送 
  SendSeg7_MESSAGE_dstAddr.endPoint = CentreApp_ENDPOINT;   //发送到目标设备的端点10上 
  SendSeg7_MESSAGE_dstAddr.addr.shortAddr = 0xFFFF;  //广播发送 
	
  if( AF_DataRequest( &SendSeg7_MESSAGE_dstAddr, &CentreApp_epDesc,
                       CentreApp_CLUSTERID,
                       len,
                       buf,
                       &CentreApp_TransID,
                       AF_DISCV_ROUTE, AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
  {
    // 发送成功后闪烁 link灯 
     HalLedBlink( HAL_LED_2, 5, 50, 250 );
     osal_memset(buf, 0, 128);  //清除缓冲区
  }
  else
  {
    // Error occurred in request to send.
  }

}





