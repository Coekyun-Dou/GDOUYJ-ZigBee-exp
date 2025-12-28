
#include "OSAL.h"
#include "AF.h"
#include "ZDApp.h"
#include "ZDObject.h"
#include "ZDProfile.h"

#include "Sensor.h"  //包含初始化数码管的函数和封装了可以直接显示数字的函数再该头文件里面

#include "seg7RouterApp.h"
#include "DebugTrace.h"

#if !defined( WIN32 )
  #include "OnBoard.h"
#endif

/* HAL */
#include "hal_lcd.h"
#include "hal_led.h"
#include "hal_key.h"
#include "hal_uart.h"


// This list should be filled with Application specific Cluster IDs.
const cId_t seg7RouterApp_ClusterList[seg7RouterApp_MAX_CLUSTERS] =
{
  	seg7RouterApp_CLUSTERID,
  	seg7RouterApp_body_CLUSTERID,           // 人体感应CID=1
    seg7RouterApp_temperature_CLUSTERID,   // 温湿度CID=2
    UART_INPUT_Control_SendSeg7_CLUSTERID
    
};

const SimpleDescriptionFormat_t seg7RouterApp_SimpleDesc =
{
  seg7RouterApp_ENDPOINT,              //  int Endpoint;
  seg7RouterApp_PROFID,                //  uint16 AppProfId[2];
  seg7RouterApp_DEVICEID,              //  uint16 AppDeviceId[2];
  seg7RouterApp_DEVICE_VERSION,        //  int   AppDevVer:4;
  seg7RouterApp_FLAGS,                 //  int   AppFlags:4;
  seg7RouterApp_MAX_CLUSTERS,          //  byte  AppNumInClusters;
  (cId_t *)seg7RouterApp_ClusterList,  //  byte *pAppInClusterList;
  seg7RouterApp_MAX_CLUSTERS,          //  byte  AppNumInClusters;
  (cId_t *)seg7RouterApp_ClusterList   //  byte *pAppInClusterList;
};

endPointDesc_t seg7RouterApp_epDesc;//端点描述符 

byte seg7RouterApp_TaskID;   // Task ID for internal task/event processing
                          // This variable will be received when
                          // seg7RouterApp_Init() is called.
devStates_t seg7RouterApp_NwkState;

byte seg7RouterApp_TransID;  // This is the unique message ID (counter)

afAddrType_t seg7RouterApp_DstAddr;


void seg7RouterApp_ProcessZDOMsgs( zdoIncomingMsg_t *inMsg );
void seg7RouterApp_HandleKeys( byte shift, byte keys );
void seg7RouterApp_MessageMSGCB( afIncomingMSGPacket_t *pckt );
void seg7RouterApp_SendTheMessage( void );
void  seg7RouterApp_Send_success_TheMessage(void);

//事件初始化函数 
void seg7RouterApp_Init( byte task_id )
{
  seg7RouterApp_TaskID = task_id;
  seg7RouterApp_NwkState = DEV_INIT;
  seg7RouterApp_TransID = 0;

  
  // 1. 初始化数码管（复用Sensor.c的IO配置）
    SensorInit();
  
  // Device hardware initialization can be added here or in main() (Zmain.c).
  // If the hardware is application specific - add it here.
  // If the hardware is other parts of the device add it in main().

  seg7RouterApp_DstAddr.addrMode = (afAddrMode_t)AddrNotPresent;
  seg7RouterApp_DstAddr.endPoint = 0;
  seg7RouterApp_DstAddr.addr.shortAddr = 0;


//以下是已经注册好的端点号为10的端点描述符 
  // Fill out the endpoint description.
  seg7RouterApp_epDesc.endPoint = seg7RouterApp_ENDPOINT;
  seg7RouterApp_epDesc.task_id = &seg7RouterApp_TaskID;
  seg7RouterApp_epDesc.simpleDesc
            = (SimpleDescriptionFormat_t *)&seg7RouterApp_SimpleDesc;
  seg7RouterApp_epDesc.latencyReq = noLatencyReqs;

  // Register the endpoint description with the AF
  afRegister( &seg7RouterApp_epDesc );

  // Register for all key events - This app will handle all key events
  RegisterForKeys( seg7RouterApp_TaskID );

  // Update the display
#if defined ( LCD_SUPPORTED )
    HalLcdWriteString( "seg7RouterApp", HAL_LCD_LINE_1 );
#endif

  ZDO_RegisterForZDOMsg( seg7RouterApp_TaskID, End_Device_Bind_rsp );
  ZDO_RegisterForZDOMsg( seg7RouterApp_TaskID, Match_Desc_rsp );
}


//事件处理函数 
UINT16 seg7RouterApp_ProcessEvent( byte task_id, UINT16 events )
{
  afIncomingMSGPacket_t *MSGpkt;
  afDataConfirm_t *afDataConfirm;

  byte sentEP;
  ZStatus_t sentStatus;
  byte sentTransID;       // This should match the value sent
  (void)task_id;  // Intentionally unreferenced parameter

  if ( events & SYS_EVENT_MSG ) //采集到事件消息 
  {
    MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( seg7RouterApp_TaskID );
    while ( MSGpkt )
    {
      switch ( MSGpkt->hdr.event )
      {
        case ZDO_CB_MSG: //收到ZDO_CB_MSG事件 （设备加入）
          seg7RouterApp_ProcessZDOMsgs( (zdoIncomingMsg_t *)MSGpkt );
          break;

        case KEY_CHANGE: //事件是KEY_CHANGE （物理按键） 
          seg7RouterApp_HandleKeys( ((keyChange_t *)MSGpkt)->state, ((keyChange_t *)MSGpkt)->keys );
          break;

        case AF_DATA_CONFIRM_CMD: //强制转换类型，防止编译器警告
          afDataConfirm = (afDataConfirm_t *)MSGpkt;
          sentEP = afDataConfirm->endpoint;
          sentStatus = afDataConfirm->hdr.status;
          sentTransID = afDataConfirm->transID;
          (void)sentEP;
          (void)sentTransID;

          if ( sentStatus != ZSuccess )
          {
      
          }
          break;

        case AF_INCOMING_MSG_CMD: //收到应用层信息 
          seg7RouterApp_MessageMSGCB( MSGpkt ); //发送信息 
          break;

        case ZDO_STATE_CHANGE: //组网状态改变 
          seg7RouterApp_NwkState = (devStates_t)(MSGpkt->hdr.status);
          if ( (seg7RouterApp_NwkState == DEV_ZB_COORD)
              || (seg7RouterApp_NwkState == DEV_ROUTER)
              || (seg7RouterApp_NwkState == DEV_END_DEVICE) )
          {
          	HalLedBlink(HAL_LED_2, 5, 50, 250);//加入网络成功后闪烁link灯 
          	
          }
          break;

        default:
          break;
      }

      osal_msg_deallocate( (uint8 *)MSGpkt );

      MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( seg7RouterApp_TaskID );
    }

    return (events ^ SYS_EVENT_MSG);
  }

  if ( events & seg7RouterApp_SEND_MSG_EVT ) //在看看有没有消息发送事件要处理 
  {
    // Send "the" message
    seg7RouterApp_SendTheMessage();

    // Setup to send message again
    osal_start_timerEx( seg7RouterApp_TaskID,
                        seg7RouterApp_SEND_MSG_EVT,
                        seg7RouterApp_SEND_MSG_TIMEOUT );

    // return unprocessed events
    return (events ^ seg7RouterApp_SEND_MSG_EVT);
  }

   
  return 0;
}

//处理组网信息 
void seg7RouterApp_ProcessZDOMsgs( zdoIncomingMsg_t *inMsg )
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
            seg7RouterApp_DstAddr.addrMode = (afAddrMode_t)Addr16Bit;
            seg7RouterApp_DstAddr.addr.shortAddr = pRsp->nwkAddr;
            // Take the first endpoint, Can be changed to search through endpoints
            seg7RouterApp_DstAddr.endPoint = pRsp->epList[0];

            // Light LED
            HalLedSet( HAL_LED_4, HAL_LED_MODE_ON );
          }
          osal_mem_free( pRsp );
        }
      }
      break;
  }
}

//物理按键 
void seg7RouterApp_HandleKeys( byte shift, byte keys )
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
                            seg7RouterApp_epDesc.endPoint,
                            seg7RouterApp_PROFID,
                            seg7RouterApp_MAX_CLUSTERS, (cId_t *)seg7RouterApp_ClusterList,
                            seg7RouterApp_MAX_CLUSTERS, (cId_t *)seg7RouterApp_ClusterList,
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
                        seg7RouterApp_PROFID,
                        seg7RouterApp_MAX_CLUSTERS, (cId_t *)seg7RouterApp_ClusterList,
                        seg7RouterApp_MAX_CLUSTERS, (cId_t *)seg7RouterApp_ClusterList,
                        FALSE );
    }
  }
}

//接收信息 
void seg7RouterApp_MessageMSGCB( afIncomingMSGPacket_t *pkt )
{
  switch ( pkt->clusterId )
  {
    
  case seg7RouterApp_CLUSTERID:
       break; 
      
  case seg7RouterApp_body_CLUSTERID:
    { DisplayTwoDigits(4,0 );   //如果收到来自于协调器发来的关于人体感应的要操作的命令就让数码管亮
     seg7RouterApp_Send_success_TheMessage();  //数码管回显给协调器成功显示的数据 
      break;  
    } 
    
  case seg7RouterApp_temperature_CLUSTERID:
    { DisplayTwoDigits(4, 0);//如果收到来自于协调器发来的关于温湿度的要操作的命令就让数码管亮
       seg7RouterApp_Send_success_TheMessage();
       // 将日志逻辑归入该case分支（避免悬空）
#if defined( LCD_SUPPORTED )
       HalLcdWriteScreen( (char*)pkt->cmd.Data, "rcvd" );
#elif defined( WIN32 )
       WPRINTSTR( pkt->cmd.Data );
#endif
       break;  
    }
    
    case UART_INPUT_Control_SendSeg7_CLUSTERID:   //协调器串口控制簇 
    {
      if( pkt->cmd.Data[7]==0xA2)
      { int receive_display_numbers = (int)pkt->cmd.Data[15];  //取出发来要显示的数字，这里假设了发来的数据是按照通信规程发来的数据。需要注意的是，如果发来的数据是不按照通信规程的话对应应该为 char receive_display_numbers= (char)pkt->cmd.Data[0]，后面可以去试试 
        unsigned char tens =(receive_display_numbers >> 4) & 0x0F;   
         unsigned char units =receive_display_numbers & 0x0F;  
	     DisplayTwoDigits(units,tens);
      }       
      break;  
    } 

  }
}





//发送信息 
void seg7RouterApp_SendTheMessage( void )
{
  char theMessageData[] = "Hello World";

  if ( AF_DataRequest( &seg7RouterApp_DstAddr, &seg7RouterApp_epDesc,
                       seg7RouterApp_CLUSTERID,
                       (byte)osal_strlen( theMessageData ) + 1,
                       (byte *)&theMessageData,
                       &seg7RouterApp_TransID,
                       AF_DISCV_ROUTE, AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
  {
    // Successfully requested to be sent.
  }
  else
  {
    // Error occurred in request to send.
  }
}

void  seg7RouterApp_Send_success_TheMessage(void)  //发送成功显示的无线信号给协调器 
{

uint8 txFrame[32];
    uint8 i = 0;
    uint8 checkSum = 0;
    static uint8 responseSeq = 0; // 响应帧序号

    // 1. 获取本节点及父节点地址
    uint16 myShortAddr = NLME_GetShortAddr();
    uint16 parentShortAddr = NLME_GetCoordShortAddr();

    // 2. 构建固定帧头
    txFrame[i++] = 0xAA;               // 帧头
    txFrame[i++] = 0x00;               // 帧长度占位，最后计算长度 
    txFrame[i++] = 0x02;               // 帧类型: 无线传感数据
    txFrame[i++] = responseSeq++;      // 帧序号

    // 3. 业务数据 - 命令类型与协议类型
    txFrame[i++] = 0x03;               // ★ 关键：命令类型 为上传路由就是发送自身的信息给到协调器 
    txFrame[i++] = 0x01;               // 协议类型: Zigbee

    // 4. 传感器类型与索引 (与接收到的命令保持一致)
    txFrame[i++] = 0x00;               // 传感器类型高字节 (数码管: 0x00A2)
    txFrame[i++] = 0xA2;               // 传感器类型低字节
    txFrame[i++] = 0x00;               // 索引

    // 5. 网络地址信息，采用的是大端模式。就是把源地址的高字节放在数据帧的低字节中 
    txFrame[i++] = HI_UINT16(myShortAddr);      // 源地址高字节
    txFrame[i++] = LO_UINT16(myShortAddr);      // 源地址低字节
    txFrame[i++] = HI_UINT16(parentShortAddr);  // 父地址高字节
    txFrame[i++] = LO_UINT16(parentShortAddr);  // 父地址低字节

    txFrame[i++] = 0x01;               // 节点类型: 路由节点

    // 6. 值长度与数据值 (表示执行结果)
    txFrame[i++] = 0x01;               // 值类型: 布尔型
    // 数据值: 通常0x00表示成功，具体请参考协议文档
    txFrame[i++] =0x00;            //发00表示成功显示

    // 7. 计算帧长度并回填 (此时i=17)
    // 总长度 = 当前i(17) + 1个校验码字节 = 18 (0x12)
     txFrame[1] = i + 1;

    // 8. 计算校验和
    for (uint8 j = 0; j < i; j++) {
        checkSum += txFrame[j];
    }
    txFrame[i++] = checkSum;           // 帧尾：校验码

    // 9. 发送响应给协调器
    afAddrType_t dstAddr;
    dstAddr.addrMode = (afAddrMode_t)afAddr16Bit;
    dstAddr.endPoint = seg7RouterApp_ENDPOINT; // 这个是要发到对应协调器的端点10，端点号是10 
    dstAddr.addr.shortAddr = 0x0000; // 目标：协调器地址

    if (AF_DataRequest(&dstAddr,//决定了要发送到哪个设备的哪个端点上 ，这里是协调器设备的端点号10 
                       &seg7RouterApp_epDesc,  //这个源端点的描述符，就是说是表明本设备的是哪个端点，前面已经注册这个端口号10，可见发送到目的设备的端口号要跟自身设备的端口号相同，因为发送端和接收端必须使用相同的端点号才能通信！！ 
                       seg7RouterApp_CLUSTERID , //发送CID让接收到该信息的协调器做不同的处理，该CID是5，注意：发送端和接收端的CID要相同！！，发送端和接收端的端点号必须匹配，（比如发送到端点 10，接收端也必须在端点 10 注册了该 CID） 
                       i,       // 发送总字节数 (18)
                       txFrame,
                       &seg7RouterApp_TransID,
                       AF_DISCV_ROUTE,
                       AF_DEFAULT_RADIUS) == afStatus_SUCCESS)
	{
        HalLedBlink(HAL_LED_2, 5, 250, 50); // 发送成功的话就让data灯闪烁 
    } 
	else
	 {
        HalLedBlink(HAL_LED_1, 5, 250, 50);//data闪烁表示发送失败 
     }

}

/*
端点号（Endpoint）：Zigbee 设备中用于区分不同应用的逻辑端口（范围 1-240），发送端和接收端必须使用相同的端点号才能通信。

Cluster ID（CID）：集群 ID，用于区分同一端点下的不同业务（如温度数据、开关控制等），发送的消息会携带 CID，接收端需注册相同 CID（在seg7RouterApp_ClusterList数组中可以添加，再给端点注册就好了） 才能识别。


注册逻辑：通过协议栈的afRegister接口注册端点描述符，其中包含该端点支持的 CID 列表，协议栈会根据端点号和 CID 路由消息。

通过以上步骤，发送端发送的消息会携带正确的端点号和 CID，接收端（协调器）因注册了相同的端点和 CID，能够正常识别并处理消息。


*/







