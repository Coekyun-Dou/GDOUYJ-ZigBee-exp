
#include "OSAL.h"
#include "AF.h"
#include "ZDApp.h"
#include "ZDObject.h"
#include "ZDProfile.h"

#include "PeopleApp.h"
#include "DebugTrace.h"

#if !defined( WIN32 )
  #include "OnBoard.h"
#endif

/* HAL */
#include "hal_lcd.h"
#include "hal_led.h"
#include "hal_key.h"
#include "hal_uart.h"


//³õÊ¼»¯ 
const cId_t PeopleApp_ClusterList[PeopleApp_MAX_CLUSTERS] =
{
  PeopleApp_CLUSTERID
};

const SimpleDescriptionFormat_t PeopleApp_SimpleDesc =
{
  PeopleApp_ENDPOINT,              //  int Endpoint;
  PeopleApp_PROFID,                //  uint16 AppProfId[2];
  PeopleApp_DEVICEID,              //  uint16 AppDeviceId[2];
  PeopleApp_DEVICE_VERSION,        //  int   AppDevVer:4;
  PeopleApp_FLAGS,                 //  int   AppFlags:4;
  PeopleApp_MAX_CLUSTERS,          //  byte  AppNumInClusters;
  (cId_t *)PeopleApp_ClusterList,  //  byte *pAppInClusterList;
  PeopleApp_MAX_CLUSTERS,          //  byte  AppNumInClusters;
  (cId_t *)PeopleApp_ClusterList   //  byte *pAppInClusterList;
};

endPointDesc_t PeopleApp_epDesc;

//TASKIDÿÿ?ÿÿ
byte PeopleApp_TaskID;   // Task ID for internal task/event processing
                          // This variable will be received when
                          // PeopleApp_Init() is called.
devStates_t PeopleApp_NwkState;


byte PeopleApp_TransID;  // This is the unique message ID (counter)

afAddrType_t PeopleApp_DstAddr;

/*********************************************************************
 * ÿÿÿÿÿÿÿÿ
 */
void PeopleApp_ProcessZDOMsgs( zdoIncomingMsg_t *inMsg );
void PeopleApp_HandleKeys( byte shift, byte keys );
void PeopleApp_MessageMSGCB( afIncomingMSGPacket_t *pckt );

void  PeopleApp_Send_body_Message(void);


//ÿÿÿÿÿ?ÿÿÿÿ
void PeopleApp_Init( byte task_id )
{
  PeopleApp_TaskID = task_id;
  PeopleApp_NwkState = DEV_INIT;
  PeopleApp_TransID = 0;
  
  //ÿÿÿÿÿ?ÿÿ?ÿÿ 
   P1DIR&=~(0x01<<0);//ÿÿÿÿP1.0?ÿÿÿ??
   P1SEL&=0x01;//ÿÿÿÿP1.0??ÿÿÿÿÿ?ÿÿ
   

  PeopleApp_DstAddr.addrMode = (afAddrMode_t)AddrNotPresent;
  PeopleApp_DstAddr.endPoint = 0;
  PeopleApp_DstAddr.addr.shortAddr = 0;

  // Fill out the endpoint description.
  PeopleApp_epDesc.endPoint = PeopleApp_ENDPOINT;
  PeopleApp_epDesc.task_id = &PeopleApp_TaskID;
  PeopleApp_epDesc.simpleDesc
            = (SimpleDescriptionFormat_t *)&PeopleApp_SimpleDesc;
  PeopleApp_epDesc.latencyReq = noLatencyReqs;

  // Register the endpoint description with the AF
  afRegister( &PeopleApp_epDesc );

  // Register for all key events - This app will handle all key events
  RegisterForKeys( PeopleApp_TaskID );

  // Update the display
#if defined ( LCD_SUPPORTED )
    HalLcdWriteString( "PeopleApp", HAL_LCD_LINE_1 );
#endif

  ZDO_RegisterForZDOMsg( PeopleApp_TaskID, End_Device_Bind_rsp );
  ZDO_RegisterForZDOMsg( PeopleApp_TaskID, Match_Desc_rsp );
}

//ÿ?ÿÿÿÿÿÿÿÿÿ
UINT16 PeopleApp_ProcessEvent( byte task_id, UINT16 events )
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
    MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( PeopleApp_TaskID );
    while ( MSGpkt )
    {
      switch ( MSGpkt->hdr.event )
      {
        case ZDO_CB_MSG:
          PeopleApp_ProcessZDOMsgs( (zdoIncomingMsg_t *)MSGpkt );
          break;

        case KEY_CHANGE:
          PeopleApp_HandleKeys( ((keyChange_t *)MSGpkt)->state, ((keyChange_t *)MSGpkt)->keys );
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
          PeopleApp_MessageMSGCB( MSGpkt );
          break;

        case ZDO_STATE_CHANGE:
          PeopleApp_NwkState = (devStates_t)(MSGpkt->hdr.status);
          if ( (PeopleApp_NwkState == DEV_ZB_COORD)
              || (PeopleApp_NwkState == DEV_ROUTER)
              || (PeopleApp_NwkState == DEV_END_DEVICE) )
          {
             HalLedBlink( HAL_LED_1, 5, 50, 250 );
            // Start sending "the" message in a regular interval.
          #if!defined(ZOD_COORDINATOR)                                //ÿÿÿÿÿÿ???ÿÿÿ?ÿ?ÿ?ÿÿÿÿÿÿÿÿÿ?ÿÿÿÿ
            osal_start_timerEx( PeopleApp_TaskID,
                                PeopleApp_SEND_MSG_PERIODIC_BODY_EVT,
                                PeopleApp_SEND_MSG_TIMEOUT ); 
          #endif  
            
          }
          break;

        default:
          break;
      }

      // Release the memory
      osal_msg_deallocate( (uint8 *)MSGpkt );

      // Next
      MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( PeopleApp_TaskID );
    }

    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }

  // Send a message out - This event is generated by a timer
  //  (setup in PeopleApp_Init()).
  if ( events & PeopleApp_SEND_MSG_EVT )
  {
    // Send "the" message
   //PeopleApp_SendTheMessage();

    // Setup to send message again
    /*osal_start_timerEx( PeopleApp_TaskID,
                        PeopleApp_SEND_MSG_EVT,
                        PeopleApp_SEND_MSG_TIMEOUT );*/

    // return unprocessed events
    return (events ^ PeopleApp_SEND_MSG_EVT);
  }
  
  if ( events & PeopleApp_SEND_MSG_PERIODIC_BODY_EVT )
  {
    

    
      PeopleApp_Send_body_Message();

    
    // Setup to send message again
    osal_start_timerEx( PeopleApp_TaskID,
                        PeopleApp_SEND_MSG_PERIODIC_BODY_EVT,
                        PeopleApp_SEND_MSG_TIMEOUT );

    // return unprocessed events
    return (events ^ PeopleApp_SEND_MSG_PERIODIC_BODY_EVT);
  }
 
  
  
  return 0;
}



/*********************************************************************
 * Event Generation Functions
 */

/*********************************************************************
 * @fn      PeopleApp_ProcessZDOMsgs()
 *
 * @brief   Process response messages
 *
 * @param   none
 *
 * @return  none
 */
void PeopleApp_ProcessZDOMsgs( zdoIncomingMsg_t *inMsg )
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
            PeopleApp_DstAddr.addrMode = (afAddrMode_t)Addr16Bit;
            PeopleApp_DstAddr.addr.shortAddr = pRsp->nwkAddr;
            // Take the first endpoint, Can be changed to search through endpoints
            PeopleApp_DstAddr.endPoint = pRsp->epList[0];

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
 * @fn      PeopleApp_HandleKeys
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
void PeopleApp_HandleKeys( byte shift, byte keys )
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
                            PeopleApp_epDesc.endPoint,
                            PeopleApp_PROFID,
                            PeopleApp_MAX_CLUSTERS, (cId_t *)PeopleApp_ClusterList,
                            PeopleApp_MAX_CLUSTERS, (cId_t *)PeopleApp_ClusterList,
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
                        PeopleApp_PROFID,
                        PeopleApp_MAX_CLUSTERS, (cId_t *)PeopleApp_ClusterList,
                        PeopleApp_MAX_CLUSTERS, (cId_t *)PeopleApp_ClusterList,
                        FALSE );
    }
  }
}

/*********************************************************************
 * LOCAL FUNCTIONS
 */

/*********************************************************************
 * @fn      PeopleApp_MessageMSGCB
 *
 * @brief   Data message processor callback.  This function processes
 *          any incoming data - probably from other devices.  So, based
 *          on cluster ID, perform the intended action.
 *
 * @param   none
 *
 * @return  none
 */
void PeopleApp_MessageMSGCB( afIncomingMSGPacket_t *pkt )
{
  switch ( pkt->clusterId )
  {
    case PeopleApp_CLUSTERID:
      // "the" message
#if defined( LCD_SUPPORTED )
      HalLcdWriteScreen( (char*)pkt->cmd.Data, "rcvd" );
#elif defined( WIN32 )
      WPRINTSTR( pkt->cmd.Data );
#endif
      break;
  }
}

/*********************************************************************
 * @fn      PeopleApp_SendTheMessage
 *
 * @brief   Send "the" message.
 *
 * @param   none
 *
 * @return  none
 */

/*********************************************************************
*********************************************************************/
void  PeopleApp_Send_body_Message(void)
{
       // 1. ÿÿÿÿÿÿÿÿ?ÿÿÿ??ÿ?ÿÿ??1 + ÿÿÿÿ1 + ÿÿÿÿ1 + ÿÿÿ1 + ?ÿÿÿÿÿÿN + ?ÿÿ1ÿÿ
    uint8 txFrame[32]; // ÿÿÿÿÿ?ÿ?ÿ
    uint8 i = 0;       // ÿÿÿÿÿÿÿÿ
    uint8 checkSum = 0;
    static uint8 frameSeq = 0; // ÿÿ?ÿÿÿÿÿÿÿÿ?ÿÿ?ÿ?ÿÿ?ÿÿÿ?ÿ

    // 1. ÿÿ?ÿÿ?ÿÿÿÿÿ? (?ÿ?ÿÿ?ÿÿÿÿÿÿÿ?ÿÿÿÿÿÿÿÿ?ÿÿÿÿÿ??)
    uint16 myShortAddr = NLME_GetShortAddr();          // ÿÿÿ?ÿ?ÿ?
    uint16 parentShortAddr = NLME_GetCoordShortAddr(); // ÿÿÿ?ÿ?ÿ?



    // 2. ÿÿÿÿÿ?ÿ??ÿÿÿÿ
    txFrame[i++] = 0xAA;           // ??ÿ?ÿ?0xAA
    txFrame[i++] = 0x00;           // ?ÿÿ1ÿÿ?ÿÿÿÿÿ??ÿÿÿÿÿ0ÿÿÿÿÿÿÿÿ
    txFrame[i++] = 0x02;           // ?ÿÿ2ÿÿ?ÿÿÿ?ÿ0x02=ÿÿÿ?ÿÿÿÿÿÿÿ
    txFrame[i++] = frameSeq++;     // ?ÿÿ3ÿÿ?ÿÿ?ÿ?ÿ?ÿÿ?ÿÿÿÿÿ
    
    // 3. ÿÿÿÿ?ÿÿÿÿÿ?ÿÿ?ÿÿÿÿÿÿÿÿÿ: 0x03=ÿ?ÿÿÿÿÿÿÿÿÿ?ÿÿ?ÿ?ÿ
    txFrame[i++] = 0x03;           // ?ÿÿ4ÿÿÿÿÿÿÿÿÿÿ
    txFrame[i++] = 0x01;           // ?ÿÿ5ÿÿ?ÿÿÿÿÿ?ÿ0x01=Zigbee
    
    // ÿÿÿÿÿÿÿÿÿ?ÿÿÿÿÿÿ? = 0x0006
    txFrame[i++] = 0x00;           // ?ÿÿ6ÿÿÿÿÿÿÿÿÿÿÿ?ÿÿ?ÿ
    txFrame[i++] = 0x06;           // ?ÿÿ7ÿÿÿÿÿÿÿÿÿÿÿ?ÿÿ?ÿ
    
    txFrame[i++] = 0x00;           // ?ÿÿ8ÿÿÿÿÿÿÿ?ÿ?ÿÿ?0

    // 4. ÿÿÿ??ÿÿ?ÿÿÿÿÿÿÿ? (?ÿÿZigbee?ÿÿ?ÿ?ÿ?ÿ?)
    // ?ÿÿ? (ÿÿÿ?ÿÿ?)
   txFrame[i++] = HI_UINT16(myShortAddr);      // ?ÿÿ9ÿÿ?ÿÿ?ÿÿÿ?ÿ
    txFrame[i++] = LO_UINT16(myShortAddr);      // ?ÿÿ10ÿÿ?ÿÿ?ÿÿÿ?ÿ
    // ÿÿÿÿ?
    txFrame[i++] = HI_UINT16(parentShortAddr);  // ?ÿÿ11ÿÿÿÿÿÿ?ÿÿÿ?ÿ
    txFrame[i++] = LO_UINT16(parentShortAddr);  // ?ÿÿ12ÿÿÿÿÿÿ?ÿÿÿ?ÿ

    // 5. ÿ?ÿÿÿÿÿ (ÿÿÿÿÿ?ÿÿÿ?????? 0x02)
    txFrame[i++] = 0x02;               // ?ÿÿ13ÿÿÿ?ÿÿÿÿÿ

    // 6. ?ÿÿÿÿÿÿÿÿÿÿ?
    txFrame[i++] = 0x01;               // ?ÿÿ14ÿÿ?ÿÿÿÿ: ÿÿÿÿÿÿ

    // ÿÿ?ÿÿÿÿÿÿ??
    if (P1_0 == 0) {
        txFrame[i++] = 0x01;           // ?ÿÿ15ÿÿÿÿÿÿ?ÿÿÿÿÿÿ
    } else {
        txFrame[i++] = 0x00;           // ?ÿÿ15ÿÿÿÿÿÿ?ÿÿÿÿÿÿ
    }

    // 7. ÿÿÿ?ÿÿÿ?ÿÿÿÿ (ÿÿ?i=16ÿÿÿÿ?ÿÿÿÿ16ÿÿÿ?ÿ)
    //    ÿÿ?ÿÿ = ÿÿ?ÿ?ÿÿÿ(16) + 1ÿÿ?ÿÿÿÿÿ?ÿ = 17 (0x11)
    txFrame[1] = i + 1;                // ÿÿÿÿ?ÿÿÿÿ

    // 8. ÿÿÿÿ?ÿÿÿ
    for (uint8 j = 0; j < i; j++) {
        checkSum += txFrame[j];
    }
    txFrame[i++] = checkSum;           // ?ÿÿ16ÿÿ?ÿÿÿÿ

    // 7. ÿÿÿÿÿÿÿÿ?
      afAddrType_t    PeopleApp_Send_body_dstAddr;
      PeopleApp_Send_body_dstAddr.addrMode =(afAddrMode_t)afAddr16Bit;
      PeopleApp_Send_body_dstAddr.endPoint=PeopleApp_ENDPOINT;
      PeopleApp_Send_body_dstAddr.addr.shortAddr = 0x0000; // ?ÿÿÿ?ÿÿ?ÿÿÿÿ


 if ( AF_DataRequest(&PeopleApp_Send_body_dstAddr,
                   &PeopleApp_epDesc,
                  PeopleApp_CLUSTERID, // ?ÿÿÿÿPeopleApp.hÿÿÿ?ÿÿÿÿ??ÿID,ÿÿÿ?ÿÿ?ÿIDÿÿ?ÿÿÿÿ?ÿÿÿÿÿÿÿ?ÿÿÿ?ÿÿÿ?ÿÿÿÿÿÿ?ÿÿÿÿÿ?ÿÿÿÿÿÿÿÿ?ÿÿ?ÿIDÿÿ?ÿÿÿ?ÿ?ÿÿÿ.ÿÿ??ÿÿ?ÿÿÿ?ÿÿÿÿÿÿÿ?ÿÿ???ÿÿÿÿÿÿÿ?ÿÿÿ?ÿÿ?ÿÿÿÿÿÿapp.hÿ?ÿÿÿÿ?ÿÿ?ÿIDÿÿÿÿÿÿÿ?ÿ?ÿ?ÿÿÿ
                   i,       // ÿÿÿ?ÿÿÿÿ?ÿÿÿ
                   txFrame, // ÿÿÿÿ?ÿÿÿÿ
                   &PeopleApp_TransID,  //ÿÿÿ?ÿ?ÿÿÿ?ÿÿÿÿ??ÿÿÿ??ÿÿÿÿÿ?ÿ?ÿ??ÿÿÿÿÿ?ÿ?ÿÿÿÿÿappÿÿ?ÿÿÿÿ?ÿÿÿ?ÿÿÿ??0ÿÿ
                   AF_DISCV_ROUTE,
                   AF_DEFAULT_RADIUS ) == afStatus_SUCCESS)
  {
    // ÿÿÿ??ÿÿÿÿÿ? dataÿÿ
     
      HalLedBlink( HAL_LED_1, 5, 50, 250 );//ÿÿÿÿÿÿ5ÿÿ
  }
  else
  {
        HalLedSet( HAL_LED_2, HAL_LED_MODE_ON );
         
  }

}
