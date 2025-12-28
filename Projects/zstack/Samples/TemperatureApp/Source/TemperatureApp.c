
#include "OSAL.h"
#include "AF.h"
#include "ZDApp.h"
#include "ZDObject.h"
#include "ZDProfile.h"

#include "TemperatureApp.h"
#include "DebugTrace.h"

#if !defined( WIN32 )
  #include "OnBoard.h"
#endif

/* HAL */
#include "hal_lcd.h"
#include "hal_led.h"
#include "hal_key.h"
#include "hal_uart.h"

#include "i2c.h"
#include "Sensor.h"


// This list should be filled with Application specific Cluster IDs.
const cId_t TemperatureApp_ClusterList[TemperatureApp_MAX_CLUSTERS] =
{
  TemperatureApp_CLUSTERID
};

const SimpleDescriptionFormat_t TemperatureApp_SimpleDesc =
{
  TemperatureApp_ENDPOINT,              //  int Endpoint;
  TemperatureApp_PROFID,                //  uint16 AppProfId[2];
  TemperatureApp_DEVICEID,              //  uint16 AppDeviceId[2];
  TemperatureApp_DEVICE_VERSION,        //  int   AppDevVer:4;
  TemperatureApp_FLAGS,                 //  int   AppFlags:4;
  TemperatureApp_MAX_CLUSTERS,          //  byte  AppNumInClusters;
  (cId_t *)TemperatureApp_ClusterList,  //  byte *pAppInClusterList;
  TemperatureApp_MAX_CLUSTERS,          //  byte  AppNumInClusters;
  (cId_t *)TemperatureApp_ClusterList   //  byte *pAppInClusterList;
};

endPointDesc_t TemperatureApp_epDesc;


byte TemperatureApp_TaskID;   // Task ID for internal task/event processing
                          // This variable will be received when
                          // TemperatureApp_Init() is called.
devStates_t TemperatureApp_NwkState;


byte TemperatureApp_TransID;  // This is the unique message ID (counter)

afAddrType_t TemperatureApp_DstAddr;

/*********************************************************************
 * LOCAL FUNCTIONS
 */
void TemperatureApp_ProcessZDOMsgs( zdoIncomingMsg_t *inMsg );
void TemperatureApp_HandleKeys( byte shift, byte keys );
void TemperatureApp_MessageMSGCB( afIncomingMSGPacket_t *pckt );
void TemperatureApp_SendTheMessage( void );

//函数初始化 
void TemperatureApp_Init( byte task_id )
{
  TemperatureApp_TaskID = task_id;
  TemperatureApp_NwkState = DEV_INIT;
  TemperatureApp_TransID = 0;

  // Device hardware initialization can be added here or in main() (Zmain.c).
  // If the hardware is application specific - add it here.
  // If the hardware is other parts of the device add it in main().

  
     I2c_Init(); 
    // 初始化温湿度传感器 (地址0x44)
     SHT3X_Init(0x44);
     
    SHT3X_StartPeriodicMeasurment(REPEATAB_HIGH, FREQUENCY_2HZ); 
  
  
  
  TemperatureApp_DstAddr.addrMode = (afAddrMode_t)AddrNotPresent;
  TemperatureApp_DstAddr.endPoint = 0;
  TemperatureApp_DstAddr.addr.shortAddr = 0;

  // Fill out the endpoint description.
  TemperatureApp_epDesc.endPoint = TemperatureApp_ENDPOINT;
  TemperatureApp_epDesc.task_id = &TemperatureApp_TaskID;
  TemperatureApp_epDesc.simpleDesc
            = (SimpleDescriptionFormat_t *)&TemperatureApp_SimpleDesc;
  TemperatureApp_epDesc.latencyReq = noLatencyReqs;

  // 在AF应用框架层注册端点描述符 
  afRegister( &TemperatureApp_epDesc );

  // 事件注册 
  RegisterForKeys( TemperatureApp_TaskID );

  // Update the display
#if defined ( LCD_SUPPORTED )
    HalLcdWriteString( "TemperatureApp", HAL_LCD_LINE_1 );
#endif

  ZDO_RegisterForZDOMsg( TemperatureApp_TaskID, End_Device_Bind_rsp );
  ZDO_RegisterForZDOMsg( TemperatureApp_TaskID, Match_Desc_rsp );
}

//事件处理函数 
UINT16 TemperatureApp_ProcessEvent( byte task_id, UINT16 events )
{
  afIncomingMSGPacket_t *MSGpkt; //接收数据包 
  afDataConfirm_t *afDataConfirm; //数据确认消息 

  // 声明数据端点、发送状态以及事务id 
  byte sentEP;
  ZStatus_t sentStatus;
  byte sentTransID;       
  (void)task_id;  

  if ( events & SYS_EVENT_MSG ) //采集到事件消息 
  {
  	//把消息存到MSGpkt中 
    MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( TemperatureApp_TaskID );
    while ( MSGpkt )
    {
      switch ( MSGpkt->hdr.event ) 
      {
        case ZDO_CB_MSG: //收到ZDO_CB_MSG事件 （设备加入） 
          TemperatureApp_ProcessZDOMsgs( (zdoIncomingMsg_t *)MSGpkt ); //调用 
          break;

        case KEY_CHANGE: //事件是KEY_CHANGE 
          TemperatureApp_HandleKeys( ((keyChange_t *)MSGpkt)->state, ((keyChange_t *)MSGpkt)->keys );
          break;

        case AF_DATA_CONFIRM_CMD:  
		//强制转换类型，防止编译器警告 
          afDataConfirm = (afDataConfirm_t *)MSGpkt;
          sentEP = afDataConfirm->endpoint;
          sentStatus = afDataConfirm->hdr.status;
          sentTransID = afDataConfirm->transID;
          (void)sentEP;
          (void)sentTransID;

          // 检查是否发送成功 
          if ( sentStatus != ZSuccess )
          {
          	
          }
          break;

        case AF_INCOMING_MSG_CMD: //有温度信息 
          TemperatureApp_MessageMSGCB( MSGpkt );//调用 
          break;

        case ZDO_STATE_CHANGE: //组网信息改变 
          TemperatureApp_NwkState = (devStates_t)(MSGpkt->hdr.status);
          if ( (TemperatureApp_NwkState == DEV_ZB_COORD)
              || (TemperatureApp_NwkState == DEV_ROUTER)
              || (TemperatureApp_NwkState == DEV_END_DEVICE) )
          {
            #if!defined(ZDO_COORDINATOR)
            //组网成功后闪烁data灯
            HalLedBlink(HAL_LED_1, 5, 50, 250);  
            // Start sending "the" message in a regular interval.
            osal_start_timerEx( TemperatureApp_TaskID,
                                TemperatureApp_SEND_MSG_EVT,
                                TemperatureApp_SEND_MSG_TIMEOUT );
            #endif
          
          }
          break;

        default:
          break;
      }

      // Release the memory
      osal_msg_deallocate( (uint8 *)MSGpkt );

      // Next
      MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( TemperatureApp_TaskID );
    }

    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }

  // Send a message out - This event is generated by a timer
  //  (setup in TemperatureApp_Init()).
  if ( events & TemperatureApp_SEND_MSG_EVT ) //检查是否有TemperatureApp_SEND_MSG_EVT要处理 
  {
    // 发送信息 
    TemperatureApp_SendTheMessage();

    // osal定时器，超时后重发 
    osal_start_timerEx( TemperatureApp_TaskID,
                        TemperatureApp_SEND_MSG_EVT,
                        TemperatureApp_SEND_MSG_TIMEOUT );

    return (events ^ TemperatureApp_SEND_MSG_EVT);
  }

  
  return 0;
}

//处理组网信息 
void TemperatureApp_ProcessZDOMsgs( zdoIncomingMsg_t *inMsg )
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

    case Match_Desc_rsp: //如果是匹配描述响应 
      {
        ZDO_ActiveEndpointRsp_t *pRsp = ZDO_ParseEPListRsp( inMsg );
        if ( pRsp )
        {
          if ( pRsp->status == ZSuccess && pRsp->cnt )
          {
            TemperatureApp_DstAddr.addrMode = (afAddrMode_t)Addr16Bit;
            TemperatureApp_DstAddr.addr.shortAddr = pRsp->nwkAddr;
            // Take the first endpoint, Can be changed to search through endpoints
            TemperatureApp_DstAddr.endPoint = pRsp->epList[0];

            // Light LED
            HalLedSet( HAL_LED_4, HAL_LED_MODE_ON );
          }
          osal_mem_free( pRsp );
        }
      }
      break;
  }
}


//用于支持按键输入 
void TemperatureApp_HandleKeys( byte shift, byte keys )
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
                            TemperatureApp_epDesc.endPoint,
                            TemperatureApp_PROFID,
                            TemperatureApp_MAX_CLUSTERS, (cId_t *)TemperatureApp_ClusterList,
                            TemperatureApp_MAX_CLUSTERS, (cId_t *)TemperatureApp_ClusterList,
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
                        TemperatureApp_PROFID,
                        TemperatureApp_MAX_CLUSTERS, (cId_t *)TemperatureApp_ClusterList,
                        TemperatureApp_MAX_CLUSTERS, (cId_t *)TemperatureApp_ClusterList,
                        FALSE );
    }
  }
}

//接收信息 
void TemperatureApp_MessageMSGCB( afIncomingMSGPacket_t *pkt )
{
  switch ( pkt->clusterId )
  {
    case TemperatureApp_CLUSTERID:
#if defined( LCD_SUPPORTED )
      HalLcdWriteScreen( (char*)pkt->cmd.Data, "rcvd" ); //LCD显示屏 
#elif defined( WIN32 )
      WPRINTSTR( pkt->cmd.Data ); 
#endif
      break;
  }
}

//发送信息 
void TemperatureApp_SendTheMessage( void )
{
  
   uint8 txFrame[32];
    uint8 i = 0;
    uint8 checkSum = 0;
    static uint8 frameSeq = 0;
    float temperature, humidity;
    uint16 tempValue, humiValue;

    // 1. 动态获取网络地址
    uint16 myShortAddr = NLME_GetShortAddr();
    uint16 parentShortAddr = NLME_GetCoordShortAddr();

    // 2. 读取温湿度传感器数据 (基于您提供的SHT30驱动)
    // 假设 SHT3X_ReadMeasurementBuffer 返回 0 表示成功
    if (SHT3X_ReadMeasurementBuffer(&temperature, &humidity) == 0)
    {
        // 协议要求：温度值 = 实际温度(℃) * 100， 湿度值 = 实际湿度(%RH) * 100
        // 转换为整数，并限制范围  temperature就是温度 
        tempValue = (uint16)temperature * 100;
        humiValue = (uint16)humidity * 100;
        
        // 【可选】调试打印：通过串口输出原始值
        // char dbgBuf[64];
        // halSprintf(dbgBuf, "Temp: %.2fC, Humi: %.2f%% -> T:%u H:%u\n", 
        //            temperature, humidity, tempValue, humiValue);
        // HalUARTWrite(0, dbgBuf, osal_strlen(dbgBuf));
        
    } else {
        // 读取失败，可设置默认值或错误标志，这里用0填充
        tempValue = 0;
        humiValue = 0;
    }

    // 3. 构建协议数据帧
    // 固定帧头
    txFrame[i++] = 0xAA;               // 帧头
    txFrame[i++] = 0x00;               // 帧长度占位
    txFrame[i++] = 0x02;               // 帧类型: 无线传感数据
    txFrame[i++] = frameSeq++;         // 帧序号

    // 业务数据头
    txFrame[i++] = 0x03;               // 命令类型: 上传路由信息 (0x03)
    txFrame[i++] = 0x01;               // 协议类型: Zigbee (0x01)
    
    // 传感器类型: 温湿度 = 0x0002
    txFrame[i++] = 0x00;               // 高字节
    txFrame[i++] = 0x02;               // 低字节
    
    txFrame[i++] = 0x00;               // 索引号 (默认0)

    // 网络地址 (大端格式)
    txFrame[i++] = HI_UINT16(myShortAddr);      // 源地址高字节
    txFrame[i++] = LO_UINT16(myShortAddr);      // 源地址低字节
    txFrame[i++] = HI_UINT16(parentShortAddr);  // 父地址高字节
    txFrame[i++] = LO_UINT16(parentShortAddr);  // 父地址低字节

    txFrame[i++] = 0x02;               // 节点类型: 终端设备 (0x02)

    // 值类型与数据值 (关键部分)
    txFrame[i++] = 0xF0;               // 值类型: 特殊类型 (0xF0，对应温湿度)
    
    // 数据值: 4字节，温度(2字节) + 湿度(2字节)，注意字节顺序
    // 温度值 (例如 22.65℃ = 0x08D9=2265) 温度值是放大了100倍的 
    txFrame[i++] = HI_UINT16(tempValue); // 温度高字节，温度的高位放在数据帧的低字节 
    txFrame[i++] = LO_UINT16(tempValue); // 温度低字节， 温度的低位放在数据帧的高字节
    // 湿度值 (例如 33.69% = 0x0D29)湿度值是放大了100倍的 
    txFrame[i++] = HI_UINT16(humiValue); // 湿度高字节，湿度的高位放在数据帧的低字节 
    txFrame[i++] = LO_UINT16(humiValue); // 湿度低字节， 湿度的低位放在数据帧的高字节

    // 4. 计算帧长度并回填
    // 此时 i = 已填入的字节数 (应该是 20)
    // 总长度 = 当前i + 1个校验码字节 = 21 (0x15)
    txFrame[1] = i + 1;                // 回填帧长度到第二个字节

    // 5. 计算校验和 (从帧头到数据值最后一个字节)
    for (uint8 j = 0; j < i; j++) {
        checkSum += txFrame[j];
    }
    txFrame[i++] = checkSum;           // 填入校验码

    // 6. 发送数据帧
    afAddrType_t dstAddr;
    dstAddr.addrMode = (afAddrMode_t)afAddr16Bit;
    dstAddr.endPoint = TemperatureApp_ENDPOINT; // 这个是默认的端点号
    dstAddr.addr.shortAddr = 0x0000;        // 目标: 协调器

    if (AF_DataRequest(&dstAddr,
                       &TemperatureApp_epDesc,  // 原来就有的端点描述符
                       TemperatureApp_CLUSTERID, // 请替换为您的簇ID
                       i,                    // 发送总字节数 (21)
                       txFrame,
                       &TemperatureApp_TransID,
                       AF_DISCV_ROUTE,
                       AF_DEFAULT_RADIUS) == afStatus_SUCCESS) 
    {
        // 发送成功指示data灯闪烁
        HalLedBlink(HAL_LED_1, 6, 50, 250);
      
    } else {
        // 发送失败指示link灯闪烁
        HalLedBlink(HAL_LED_2, 5, 50, 250);
    }
  
 
}
