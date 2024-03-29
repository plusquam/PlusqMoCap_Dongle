/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : p2p_client_app.c
 * Description        : P2P Client Application
 ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/

#include "main.h"
#include "app_common.h"

#include "dbg_trace.h"

#include "ble.h"
#include "p2p_client_app.h"

#include "scheduler.h"
#include "app_ble.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/

typedef enum
{
  P2P_START_TIMER_EVT,
  P2P_STOP_TIMER_EVT,
  P2P_NOTIFICATION_INFO_RECEIVED_EVT,
} P2P_Client_Opcode_Notification_evt_t;

typedef struct
{
  uint8_t * pPayload;
  uint8_t     Length;
}P2P_Client_Data_t;

typedef struct
{
  P2P_Client_Opcode_Notification_evt_t  P2P_Client_Evt_Opcode;
  P2P_Client_Data_t DataTransfered;
}P2P_Client_App_Notification_evt_t;

typedef struct
{
  /**
   * state of the P2P Client
   * state machine
   */
  APP_BLE_ConnStatus_t state;

  /**
   * connection handle
   */
  uint16_t connHandle;

  /**
   * handle of the P2P service
   */
  uint16_t P2PServiceHandle;

  /**
   * end handle of the P2P service
   */
  uint16_t P2PServiceEndHandle;

  /**
   * handle of the Tx characteristic - Write To Server
   *
   */
  uint16_t P2PWriteToServerCharHdle;

  /**
   * handle of the client configuration
   * descriptor of Tx characteristic
   */
  uint16_t P2PWriteToServerDescHandle;

  /**
   * handle of the Rx characteristic - Notification From Server
   *
   */
  uint16_t P2PNotificationCharHdle;

  /**
   * handle of the client configuration
   * descriptor of Rx characteristic
   */
  uint16_t P2PNotificationDescHandle;

}P2P_ClientContext_t;

/* USER CODE BEGIN PTD */
typedef struct{
  uint8_t	Device_Led_Selection;
  uint8_t	Led1;
}P2P_LedCharValue_t;

typedef struct{
  uint8_t	Payload[2];
  uint8_t	Length;
}P2P_ServerCommand_t;

typedef struct
{
  uint8_t				Notification_Status; /* used to chek if P2P Server is enabled to Notify */
  P2P_LedCharValue_t	LedControl;
  P2P_ServerCommand_t	CommandToSend;
  uint16_t 				ConnectionHandle;
  uint16_t				Negotiated_ATT_MTU;

} P2P_Client_App_Context_t;

/* USER CODE END PTD */

/* Private defines ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macros -------------------------------------------------------------*/
#define UNPACK_2_BYTE_PARAMETER(ptr)  \
        (uint16_t)((uint16_t)(*((uint8_t *)ptr))) |   \
        (uint16_t)((((uint16_t)(*((uint8_t *)ptr + 1))) << 8))
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/**
 * START of Section BLE_APP_CONTEXT
 */

PLACE_IN_SECTION("BLE_APP_CONTEXT") static P2P_ClientContext_t aP2PClientContext[BLE_CFG_CLT_MAX_NBR_CB];

/**
 * END of Section BLE_APP_CONTEXT
 */
/* USER CODE BEGIN PV */
PLACE_IN_SECTION("BLE_APP_CONTEXT") static P2P_Client_App_Context_t P2P_Client_App_Context;
struct {
  uint8_t data[77];
  uint8_t length;
} P2P_measurementData;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
static void Gatt_Notification(P2P_Client_App_Notification_evt_t *pNotification);
static SVCCTL_EvtAckStatus_t Event_Handler(void *Event);
/* USER CODE BEGIN PFP */
static void P2PC_APP_Send_Command(void);
static void Update_Service();
static tBleStatus Write_Char(uint16_t UUID, uint8_t Service_Instance, uint8_t *pPayload, uint8_t length);
void P2PC_APP_Notification_Data_Send(void);
static void P2PS_Show_Config(void);
static void P2PC_APP_Call_ATT_MTU_Exchange_Command(void);
/* USER CODE END PFP */

/* Functions Definition ------------------------------------------------------*/
/**
 * @brief  Service initialization
 * @param  None
 * @retval None
 */
void P2PC_APP_Init(void)
{
  uint8_t index =0;
/* USER CODE BEGIN P2PC_APP_Init_1 */
  SCH_RegTask( CFG_TASK_SEARCH_SERVICE_ID, Update_Service );
  SCH_RegTask( CFG_TASK_SEND_COMMAND_ID, P2PC_APP_Send_Command );
  SCH_RegTask( CFG_TASK_VCP_SEND_MEASUREMENT_DATA_ID, P2PC_APP_Notification_Data_Send);
  SCH_RegTask( CFG_TASK_ATT_MTU_EXCHANGE_ID, P2PC_APP_Call_ATT_MTU_Exchange_Command);
  SCH_RegTask( CFG_TASK_SET_PHY_ID, P2PC_APP_Set_PHY);
  SCH_RegTask( CFG_TASK_READ_CFG_ID, P2PS_Show_Config );

  
	/**
	* Initialize LedButton Service
	*/
  P2P_Client_App_Context.Notification_Status=0;
  P2P_Client_App_Context.ConnectionHandle =  0x00;

  P2P_Client_App_Context.LedControl.Device_Led_Selection=0x00;/* device Led */
  P2P_Client_App_Context.LedControl.Led1=0x00; /* led OFF */

  P2P_Client_App_Context.Negotiated_ATT_MTU = 0u;
/* USER CODE END P2PC_APP_Init_1 */
  for(index = 0; index < BLE_CFG_CLT_MAX_NBR_CB; index++)
  {
    aP2PClientContext[index].state= APP_BLE_IDLE;
  }

  /**
   *  Register the event handler to the BLE controller
   */
  SVCCTL_RegisterCltHandler(Event_Handler);

#if(CFG_DEBUG_APP_TRACE != 0)
  APP_DBG_MSG("-- P2P CLIENT INITIALIZED \n");
#endif

/* USER CODE BEGIN P2PC_APP_Init_2 */

/* USER CODE END P2PC_APP_Init_2 */
  return;
}

void P2PC_APP_Notification(P2PC_APP_ConnHandle_Not_evt_t *pNotification)
{
/* USER CODE BEGIN P2PC_APP_Notification_1 */

/* USER CODE END P2PC_APP_Notification_1 */
  switch(pNotification->P2P_Evt_Opcode)
  {
/* USER CODE BEGIN P2P_Evt_Opcode */

/* USER CODE END P2P_Evt_Opcode */

  case PEER_CONN_HANDLE_EVT :
/* USER CODE BEGIN PEER_CONN_HANDLE_EVT */
	P2P_Client_App_Context.ConnectionHandle = pNotification->ConnectionHandle;

    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
/* USER CODE END PEER_CONN_HANDLE_EVT */
      break;

    case PEER_DISCON_HANDLE_EVT :
/* USER CODE BEGIN PEER_DISCON_HANDLE_EVT */
      {
      uint8_t index = 0;
      P2P_Client_App_Context.ConnectionHandle =  0x00;
      while((index < BLE_CFG_CLT_MAX_NBR_CB) &&
                  (aP2PClientContext[index].state != APP_BLE_IDLE))
      {
        aP2PClientContext[index].state = APP_BLE_IDLE;
      }
      HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);

      P2P_Client_App_Context.Negotiated_ATT_MTU = 0u;
        
#if OOB_DEMO == 0
      SCH_SetTask(1<<CFG_TASK_CONN_DEV_1_ID, CFG_SCH_PRIO_0);
#endif 
      }
/* USER CODE END PEER_DISCON_HANDLE_EVT */
      break;

    default:
/* USER CODE BEGIN P2P_Evt_Opcode_Default */

/* USER CODE END P2P_Evt_Opcode_Default */
      break;
  }
/* USER CODE BEGIN P2PC_APP_Notification_2 */

/* USER CODE END P2PC_APP_Notification_2 */
  return;
}
/* USER CODE BEGIN FD */

void P2PC_APP_Notification_Data_Send(void)
{
	P2P_measurementData.data[P2P_measurementData.length] = '\r';
	P2P_measurementData.data[P2P_measurementData.length + 1] = '\n';
	DbgTraceWrite(1, P2P_measurementData.data, P2P_measurementData.length + 2);
}

/* USER CODE END FD */

/*************************************************************
 *
 * LOCAL FUNCTIONS
 *
 *************************************************************/

/**
 * @brief  Event handler
 * @param  Event: Address of the buffer holding the Event
 * @retval Ack: Return whether the Event has been managed or not
 */
static SVCCTL_EvtAckStatus_t Event_Handler(void *Event)
{
  SVCCTL_EvtAckStatus_t return_value;
  hci_event_pckt *event_pckt;
  evt_blue_aci *blue_evt;

  P2P_Client_App_Notification_evt_t Notification;

  return_value = SVCCTL_EvtNotAck;
  event_pckt = (hci_event_pckt *)(((hci_uart_pckt*)Event)->data);

  switch(event_pckt->evt)
  {
    case EVT_VENDOR:
    {
      blue_evt = (evt_blue_aci*)event_pckt->data;
      switch(blue_evt->ecode)
      {

        case EVT_BLUE_ATT_READ_BY_GROUP_TYPE_RESP:
        {
          aci_att_read_by_group_type_resp_event_rp0 *pr = (void*)blue_evt->data;
          uint8_t numServ, i, idx;
          uint16_t uuid, handle;

          uint8_t index;
          handle = pr->Connection_Handle;
          index = 0;
          while((index < BLE_CFG_CLT_MAX_NBR_CB) &&
                  (aP2PClientContext[index].state != APP_BLE_IDLE))
          {
            APP_BLE_ConnStatus_t status;

            status = APP_BLE_Get_Client_Connection_Status(aP2PClientContext[index].connHandle);

            if((aP2PClientContext[index].state == APP_BLE_CONNECTED_CLIENT)&&
                    (status == APP_BLE_IDLE))
            {
              /* Handle deconnected */

              aP2PClientContext[index].state = APP_BLE_IDLE;
              aP2PClientContext[index].connHandle = 0xFFFF;
              break;
            }
            index++;
          }

          if(index < BLE_CFG_CLT_MAX_NBR_CB)
          {
            aP2PClientContext[index].connHandle= handle;

            numServ = (pr->Data_Length) / pr->Attribute_Data_Length;

            /* the event data will be
             * 2bytes start handle
             * 2bytes end handle
             * 2 or 16 bytes data
             * we are interested only if the UUID is 16 bit.
             * So check if the data length is 6
             */
#if (UUID_128BIT_FORMAT==1)           
          if (pr->Attribute_Data_Length == 20)
          {
            idx = 16;
#else
          if (pr->Attribute_Data_Length == 6)
          {
            idx = 4;
#endif             
              for (i=0; i<numServ; i++)
              {
                uuid = UNPACK_2_BYTE_PARAMETER(&pr->Attribute_Data_List[idx]);
                if(uuid == P2P_SERVICE_UUID)
                {
#if(CFG_DEBUG_APP_TRACE != 0)
                  APP_DBG_MSG("-- GATT : P2P_SERVICE_UUID FOUND - connection handle 0x%x \n", aP2PClientContext[index].connHandle);
#endif
#if (UUID_128BIT_FORMAT==1)                     
                aP2PClientContext[index].P2PServiceHandle = UNPACK_2_BYTE_PARAMETER(&pr->Attribute_Data_List[idx-16]);
                aP2PClientContext[index].P2PServiceEndHandle = UNPACK_2_BYTE_PARAMETER (&pr->Attribute_Data_List[idx-14]);
#else   
                aP2PClientContext[index].P2PServiceHandle = UNPACK_2_BYTE_PARAMETER(&pr->Attribute_Data_List[idx-4]);
                aP2PClientContext[index].P2PServiceEndHandle = UNPACK_2_BYTE_PARAMETER (&pr->Attribute_Data_List[idx-2]);
#endif                  
                  aP2PClientContext[index].state = APP_BLE_DISCOVER_CHARACS ;
                }
                idx += 6;
              }
            }
          }
        }
        break;

        case EVT_BLUE_ATT_READ_BY_TYPE_RESP:
        {

          aci_att_read_by_type_resp_event_rp0 *pr = (void*)blue_evt->data;
          uint8_t idx;
          uint16_t uuid, handle;

          /* the event data will be
           * 2 bytes start handle
           * 1 byte char properties
           * 2 bytes handle
           * 2 or 16 bytes data
           */

          uint8_t index;

          index = 0;
          while((index < BLE_CFG_CLT_MAX_NBR_CB) &&
                  (aP2PClientContext[index].connHandle != pr->Connection_Handle))
            index++;

          if(index < BLE_CFG_CLT_MAX_NBR_CB)
          {

            /* we are interested in only 16 bit UUIDs */
#if (UUID_128BIT_FORMAT==1)
            idx = 17;
            if (pr->Handle_Value_Pair_Length == 21)
#else
              idx = 5;
            if (pr->Handle_Value_Pair_Length == 7)
#endif
            {
              pr->Data_Length -= 1;
              while(pr->Data_Length > 0)
              {
                uuid = UNPACK_2_BYTE_PARAMETER(&pr->Handle_Value_Pair_Data[idx]);
                /* store the characteristic handle not the attribute handle */
#if (UUID_128BIT_FORMAT==1)
                handle = UNPACK_2_BYTE_PARAMETER(&pr->Handle_Value_Pair_Data[idx-14]);
#else
                handle = UNPACK_2_BYTE_PARAMETER(&pr->Handle_Value_Pair_Data[idx-2]);
#endif
                if(uuid == P2P_WRITE_CHAR_UUID)
                {
#if(CFG_DEBUG_APP_TRACE != 0)
                  APP_DBG_MSG("-- GATT : WRITE_UUID FOUND - connection handle 0x%x\n", aP2PClientContext[index].connHandle);
#endif
                  aP2PClientContext[index].state = APP_BLE_DISCOVER_WRITE_DESC;
                  aP2PClientContext[index].P2PWriteToServerCharHdle = handle;
                }

                else if(uuid == P2P_NOTIFY_CHAR_UUID)
                {
#if(CFG_DEBUG_APP_TRACE != 0)
                  APP_DBG_MSG("-- GATT : NOTIFICATION_CHAR_UUID FOUND  - connection handle 0x%x\n", aP2PClientContext[index].connHandle);
#endif
                  aP2PClientContext[index].state = APP_BLE_DISCOVER_NOTIFICATION_CHAR_DESC;
                  aP2PClientContext[index].P2PNotificationCharHdle = handle;
                }
#if (UUID_128BIT_FORMAT==1)
                pr->Data_Length -= 21;
                idx += 21;
#else
                pr->Data_Length -= 7;
                idx += 7;
#endif
              }
            }
          }
        }
        break;

        case EVT_BLUE_ATT_FIND_INFORMATION_RESP:
        {
          aci_att_find_info_resp_event_rp0 *pr = (void*)blue_evt->data;

          uint8_t numDesc, idx, i;
          uint16_t uuid, handle;

          /*
           * event data will be of the format
           * 2 bytes handle
           * 2 bytes UUID
           */

          uint8_t index;

          index = 0;
          while((index < BLE_CFG_CLT_MAX_NBR_CB) &&
                  (aP2PClientContext[index].connHandle != pr->Connection_Handle))

            index++;

          if(index < BLE_CFG_CLT_MAX_NBR_CB)
          {

            numDesc = (pr->Event_Data_Length) / 4;
            /* we are interested only in 16 bit UUIDs */
            idx = 0;
            if (pr->Format == UUID_TYPE_16)
            {
              for (i=0; i<numDesc; i++)
              {
                handle = UNPACK_2_BYTE_PARAMETER(&pr->Handle_UUID_Pair[idx]);
                uuid = UNPACK_2_BYTE_PARAMETER(&pr->Handle_UUID_Pair[idx+2]);

                if(uuid == CLIENT_CHAR_CONFIG_DESCRIPTOR_UUID)
                {
#if(CFG_DEBUG_APP_TRACE != 0)
                  APP_DBG_MSG("-- GATT : CLIENT_CHAR_CONFIG_DESCRIPTOR_UUID- connection handle 0x%x\n", aP2PClientContext[index].connHandle);
#endif
                  if( aP2PClientContext[index].state == APP_BLE_DISCOVER_NOTIFICATION_CHAR_DESC)
                  {

                    aP2PClientContext[index].P2PNotificationDescHandle = handle;
                    aP2PClientContext[index].state = APP_BLE_ENABLE_NOTIFICATION_DESC;

                  }
                }
                idx += 4;
              }
            }
          }
        }
        break; /*EVT_BLUE_ATT_FIND_INFORMATION_RESP*/

        case EVT_BLUE_GATT_NOTIFICATION:
        {
          aci_gatt_notification_event_rp0 *pr = (void*)blue_evt->data;
          uint8_t index;

          index = 0;
          while((index < BLE_CFG_CLT_MAX_NBR_CB) &&
                  (aP2PClientContext[index].connHandle != pr->Connection_Handle))
            index++;

          if(index < BLE_CFG_CLT_MAX_NBR_CB)
          {

            if ( (pr->Attribute_Handle == aP2PClientContext[index].P2PNotificationCharHdle) )
            {

              Notification.P2P_Client_Evt_Opcode = P2P_NOTIFICATION_INFO_RECEIVED_EVT;
              Notification.DataTransfered.Length = pr->Attribute_Value_Length;
              Notification.DataTransfered.pPayload = &pr->Attribute_Value[0];

              Gatt_Notification(&Notification);

              /* INFORM APPLICATION BUTTON IS PUSHED BY END DEVICE */

            }
          }
        }
        break;/* end EVT_BLUE_GATT_NOTIFICATION */

        case EVT_BLUE_GATT_PROCEDURE_COMPLETE:
        {
          aci_gatt_proc_complete_event_rp0 *pr = (void*)blue_evt->data;
#if(CFG_DEBUG_APP_TRACE != 0)
          APP_DBG_MSG("-- GATT : EVT_BLUE_GATT_PROCEDURE_COMPLETE \n");
          APP_DBG_MSG("\n");
#endif

          uint8_t index;

          index = 0;
          while((index < BLE_CFG_CLT_MAX_NBR_CB) &&
                  (aP2PClientContext[index].connHandle != pr->Connection_Handle))
            index++;

          if(index < BLE_CFG_CLT_MAX_NBR_CB)
          {
            SCH_SetTask( 1<<CFG_TASK_SEARCH_SERVICE_ID, CFG_SCH_PRIO_0);
            SCH_SetTask( 1<<CFG_TASK_ATT_MTU_EXCHANGE_ID, CFG_SCH_PRIO_0);
          }
        }
        break; /*EVT_BLUE_GATT_PROCEDURE_COMPLETE*/

        case EVT_BLUE_ATT_EXCHANGE_MTU_RESP:
        {
			aci_att_exchange_mtu_resp_event_rp0 *pr = (void*)blue_evt->data;
			P2P_Client_App_Context.Negotiated_ATT_MTU = pr->Server_RX_MTU;
#if(CFG_DEBUG_APP_TRACE != 0)
			APP_DBG_MSG("-- GATT : EVT_BLUE_ATT_EXCHANGE_MTU_RESP, ATT_MTU: %d\n", P2P_Client_App_Context.Negotiated_ATT_MTU);
			APP_DBG_MSG("\n");
#endif

#if(CFG_DEBUG_APP_TRACE != 0)
			tBleStatus result = hci_le_set_data_length(P2P_Client_App_Context.ConnectionHandle, P2P_Client_App_Context.Negotiated_ATT_MTU, 2120);

			if(result)
				APP_DBG_MSG("-- hci_le_set_data_length error\n");
#else
			(void)hci_le_set_data_length(P2P_Client_App_Context.ConnectionHandle, P2P_Client_App_Context.Negotiated_ATT_MTU, 2120);
#endif
        }
        break; /*end EVT_BLUE_ATT_EXCHANGE_MTU_RESP*/

        default:
          break;
      }
    }

    break; /* HCI_EVT_VENDOR_SPECIFIC */

    default:
      break;
  }

  return(return_value);
}/* end BLE_CTRL_Event_Acknowledged_Status_t */

void Gatt_Notification(P2P_Client_App_Notification_evt_t *pNotification)
{
/* USER CODE BEGIN Gatt_Notification_1*/

/* USER CODE END Gatt_Notification_1 */
  switch(pNotification->P2P_Client_Evt_Opcode)
  {
/* USER CODE BEGIN P2P_Client_Evt_Opcode */

/* USER CODE END P2P_Client_Evt_Opcode */

    case P2P_NOTIFICATION_INFO_RECEIVED_EVT:
/* USER CODE BEGIN P2P_NOTIFICATION_INFO_RECEIVED_EVT */
    {
    	memcpy(P2P_measurementData.data, pNotification->DataTransfered.pPayload, pNotification->DataTransfered.Length);
    	P2P_measurementData.length = pNotification->DataTransfered.Length;
    	SCH_SetTask(1<<CFG_TASK_VCP_SEND_MEASUREMENT_DATA_ID, CFG_SCH_PRIO_0);
    	HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin);
    }
/* USER CODE END P2P_NOTIFICATION_INFO_RECEIVED_EVT */
      break;

    default:
/* USER CODE BEGIN P2P_Client_Evt_Opcode_Default */

/* USER CODE END P2P_Client_Evt_Opcode_Default */
      break;
  }
/* USER CODE BEGIN Gatt_Notification_2*/

/* USER CODE END Gatt_Notification_2 */
  return;
}

uint8_t P2P_Client_APP_Get_State( void ) {
  return aP2PClientContext[0].state;
}
/* USER CODE BEGIN LF */

/**
 * @brief  Feature Characteristic update
 * @param  pFeatureValue: The address of the new value to be written
 * @retval None
 */
static tBleStatus Write_Char(uint16_t UUID, uint8_t Service_Instance, uint8_t *pPayload, uint8_t length)
{

  tBleStatus ret = BLE_STATUS_INVALID_PARAMS;
  uint8_t index;

  index = 0;
  while((index < BLE_CFG_CLT_MAX_NBR_CB) &&
          (aP2PClientContext[index].state != APP_BLE_IDLE))
  {

    switch(UUID)
    {
      case P2P_WRITE_CHAR_UUID: /* SERVER RX -- so CLIENT TX */
        ret =aci_gatt_write_without_resp(aP2PClientContext[index].connHandle,
                                         aP2PClientContext[index].P2PWriteToServerCharHdle,
										 length, /* charValueLen */
                                         pPayload);

        break;

      default:
        break;
    }
    index++;
  }

  return ret;
}/* end Write_Char() */

static void P2PC_APP_Send_Command(void)
{
  APP_DBG_MSG("-- P2P APPLICATION CLIENT : WRITE TO SERVER \n ");

  Write_Char( P2P_WRITE_CHAR_UUID, 0, P2P_Client_App_Context.CommandToSend.Payload, P2P_Client_App_Context.CommandToSend.Length);

  return;
}

static void Update_Service()
{
  uint16_t enable = 0x0001;


  uint8_t index;

  index = 0;
  while((index < BLE_CFG_CLT_MAX_NBR_CB) &&
          (aP2PClientContext[index].state != APP_BLE_IDLE))
  {


    switch(aP2PClientContext[index].state)
    {

      case APP_BLE_DISCOVER_SERVICES:
        APP_DBG_MSG("P2P_DISCOVER_SERVICES\n");
        break;
      case APP_BLE_DISCOVER_CHARACS:
        APP_DBG_MSG("* GATT : Discover P2P Characteristics\n");
        aci_gatt_disc_all_char_of_service(aP2PClientContext[index].connHandle,
                                          aP2PClientContext[index].P2PServiceHandle,
                                          aP2PClientContext[index].P2PServiceEndHandle);

        break;
      case APP_BLE_DISCOVER_WRITE_DESC: /* Not Used - No decriptor */
        APP_DBG_MSG("* GATT : Discover Descriptor of TX - Write Characteritic\n");
        aci_gatt_disc_all_char_desc(aP2PClientContext[index].connHandle,
                                    aP2PClientContext[index].P2PWriteToServerCharHdle,
                                    aP2PClientContext[index].P2PWriteToServerCharHdle+2);

        break;
      case APP_BLE_DISCOVER_NOTIFICATION_CHAR_DESC:
        APP_DBG_MSG("* GATT : Discover Descriptor of Rx - Notification Characteritic\n");
        aci_gatt_disc_all_char_desc(aP2PClientContext[index].connHandle,
                                    aP2PClientContext[index].P2PNotificationCharHdle,
                                    aP2PClientContext[index].P2PNotificationCharHdle+2);

        break;
      case APP_BLE_ENABLE_NOTIFICATION_DESC:
        APP_DBG_MSG("* GATT : Enable Server Notification\n");
        aci_gatt_write_char_desc(aP2PClientContext[index].connHandle,
                                 aP2PClientContext[index].P2PNotificationDescHandle,
                                 2,
                                 (uint8_t *)&enable);

        aP2PClientContext[index].state = APP_BLE_CONNECTED_CLIENT;
        HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);

        break;
      case APP_BLE_DISABLE_NOTIFICATION_DESC :
        APP_DBG_MSG("* GATT : Disable Server Notification\n");
        aci_gatt_write_char_desc(aP2PClientContext[index].connHandle,
                                 aP2PClientContext[index].P2PNotificationDescHandle,
                                 2,
                                 (uint8_t *)&enable);

        aP2PClientContext[index].state = APP_BLE_CONNECTED_CLIENT;

        break;
      default:
        break;
    }
    index++;
  }
  return;
}

void P2PC_APP_Call_ATT_MTU_Exchange_Command()
{
	if(!P2P_Client_App_Context.Negotiated_ATT_MTU)
	{
		HAL_Delay(100);

#if(CFG_DEBUG_APP_TRACE != 0)
		tBleStatus result = aci_gatt_exchange_config(P2P_Client_App_Context.ConnectionHandle);
		if(!result)
			APP_DBG_MSG("-- GATT : aci_gatt_exchange_config sent\n");
		else
			APP_DBG_MSG("-- GATT : aci_gatt_exchange_config error %d\n", result);
#else
		(void)aci_gatt_exchange_config(P2P_Client_App_Context.ConnectionHandle);
#endif
	}
}

static void P2PS_Show_Config(void)
{
#if(CFG_DEBUG_APP_TRACE != 0)
	uint16_t supportedMaxTxOctets;
	uint16_t supportedMaxTxTime;
	uint16_t supportedMaxRxOctets;
	uint16_t supportedMaxRxTime;
	tBleStatus result = hci_le_read_maximum_data_length(&supportedMaxTxOctets,
														&supportedMaxTxTime,
														&supportedMaxRxOctets,
														&supportedMaxRxTime);
	if(!result)
	  APP_DBG_MSG("-- CONFIG: %d, %d, %d, %d\n", supportedMaxTxOctets,
				  supportedMaxTxTime, supportedMaxRxOctets, supportedMaxRxTime);
	else
		APP_DBG_MSG("-- hci_le_read_maximum_data_length error\n");

	uint8_t TX_PHY;
	uint8_t RX_PHY;
	result = hci_le_read_phy(P2P_Client_App_Context.ConnectionHandle, &TX_PHY, &RX_PHY);
	if(!result)
	  APP_DBG_MSG("-- PHY: %d, %d\n", TX_PHY, RX_PHY);
	else
		APP_DBG_MSG("-- hci_le_read_phy error\n");

	uint16_t	HC_LE_ACL_Data_Packet_Length;
	uint8_t 	HC_Total_Num_LE_ACL_Data_Packets;
	result = hci_le_read_buffer_size(&HC_LE_ACL_Data_Packet_Length, &HC_Total_Num_LE_ACL_Data_Packets);
	if(!result)
	  APP_DBG_MSG("-- Packet length: %d, Packets nr %d\n", HC_LE_ACL_Data_Packet_Length, HC_Total_Num_LE_ACL_Data_Packets);
	else
		APP_DBG_MSG("-- hci_le_read_buffer_size error\n");
	#endif
}

void P2PC_APP_Set_PHY(void)
{
	HAL_Delay(100);

#if(CFG_DEBUG_APP_TRACE != 0)
	tBleStatus result = hci_le_set_phy(P2P_Client_App_Context.ConnectionHandle, ALL_PHYS_PREFERENCE, TX_2M_PREFERRED, RX_2M_PREFERRED, 0);

	if(result)
		APP_DBG_MSG("-- hci_le_set_phy error\n");
#else
	(void)hci_le_set_phy(P2P_Client_App_Context.ConnectionHandle, ALL_PHYS_PREFERENCE, TX_2M_PREFERRED, RX_2M_PREFERRED, 0);
#endif
}

void VCP_DataReceived( uint8_t* Buf , uint32_t *Len )
{
	// Check whether command is received
	if(*Len == 1)
	{
		switch(Buf[0])
		{
		case 'C': // Connection command
		case 'c':
		{
			if(P2P_Client_APP_Get_State () != APP_BLE_CONNECTED_CLIENT)
			{
				SCH_SetTask(1 << CFG_TASK_START_SCAN_ID, CFG_SCH_PRIO_0);
			}
			else
			{
				// Disconnection routine
			}

			APP_DBG_MSG("-- Connection command got\n\n");
		}
			break;

		default: // Generic one char command
		{
			P2P_Client_App_Context.CommandToSend.Length = 1;
			P2P_Client_App_Context.CommandToSend.Payload[0] = Buf[0];
			SCH_SetTask(1<<CFG_TASK_SEND_COMMAND_ID, CFG_SCH_PRIO_0);

			APP_DBG_MSG("-- Generic one char command sent\n\n");
		}
		}
	}

	return;
}

/* USER CODE END LF */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
