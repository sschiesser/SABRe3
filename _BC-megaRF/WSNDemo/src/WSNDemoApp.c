/**********************************************************************//**
  \file WSNDemoApp.c
  \brief

  \author
    Atmel Corporation: http://www.atmel.com \n
    Support email: avr@atmel.com

  Copyright (c) 2008-2012, Atmel Corporation. All rights reserved.
  Licensed under Atmel's Limited License Agreement (BitCloudTM).

  \internal
  History:
    13/06/07 I. Kalganova - Modified
    15/03/12 D. Kolmakov - Refactored

  Last change:
    $Id: WSNDemoApp.c 21155 2012-05-11 13:57:07Z nfomin $
**************************************************************************/

/*****************************************************************************
                              Includes section
******************************************************************************/
#include <mnUtils.h>
#include <WSNDemoApp.h>
#include <WSNVisualizer.h>
#include <taskManager.h>
#include <zdo.h>
#include <configServer.h>
#include <aps.h>
#include <mac.h>
#include <WSNCommandHandler.h>
#include <WSNMessageSender.h>
#include <WSNCoord.h>
#include <WSNRouter.h>
#include <WSNEndDevice.h>

#if APP_USE_OTAU == 1
#include <WSNZclManager.h>
#endif // APP_USE_OTAU

#if defined(_COMMISSIONING_) && (APP_USE_PDS == 1)
  #include <pdsDataServer.h>
  #include <resetReason.h>
#endif // _COMMISSIONING_ && APP_USE_PDS

/*****************************************************************************
                               Definitions section
******************************************************************************/
// Period of blinking during starting network
#define START_NETWORK_BLINKING_INTERVAL 500

// Wakeup period
#define TIMER_WAKEUP 50

/*****************************************************************************
                              Types section
******************************************************************************/
typedef enum
{
  START_BLINKING,
  STOP_BLINKING
} BlinkingAction_t;

/*****************************************************************************
                              Prototypes section
******************************************************************************/
void APL_TaskHandler(void);
void ZDO_MgmtNwkUpdateNotf(ZDO_MgmtNwkUpdateNotf_t *nwkParams);
void ZDO_WakeUpInd(void);

/*****************************************************************************
                              Global variables section
******************************************************************************/
AppNwkInfoCmdPayload_t  appNwkInfo =
{
  .softVersion     = CCPU_TO_LE32(0x01010100),
  .channelMask     = CCPU_TO_LE32(CS_CHANNEL_MASK),
  .boardInfo = {
    .boardType       = MESHBEAN_SENSORS_TYPE,
    .sensorsSize     = sizeof(appNwkInfo.boardInfo.meshbean),
  },
#if APP_USE_DEVICE_CAPTION == 1
  .deviceCaption = {
    .fieldType = DEVICE_CAPTION_FIELD_TYPE,
    .size      = APP_DEVICE_CAPTION_SIZE,
    .caption   = APP_DEVICE_CAPTION
  }
#endif /* APP_USE_DEVICE_CAPTION == 1 */
};
/*****************************************************************************
                              Local variables section
******************************************************************************/
static ZDO_ZdpReq_t leaveReq;
static ZDO_StartNetworkReq_t zdoStartReq;
static SpecialDeviceInterface_t deviceInterface;
static SimpleDescriptor_t simpleDescriptor;
static APS_RegisterEndpointReq_t endpointParams;
// Application state
static AppState_t appState = APP_INITING_STATE;
static struct
{
  uint8_t appSubTaskPosted  : 1;
  uint8_t appCmdHandlerTaskPosted  : 1;
  uint8_t appMsgSenderTaskPosted  : 1;
} appTaskFlags =
{
  .appSubTaskPosted = false,
  .appCmdHandlerTaskPosted = false,
  .appMsgSenderTaskPosted = false
};

// Timer indicating starting network
static HAL_AppTimer_t startingNetworkTimer;
static HAL_AppTimer_t identifyTimer;
static uint16_t identifyDuration = 0;

/****************************************************************************
                              Static functions prototypes section
******************************************************************************/
static void appPostGlobalTask(void);
static void appInitialization(void);
static void appZdoNwkUpdateHandler(ZDO_MgmtNwkUpdateNotf_t *updateParam);

static void appZdoStartNetworkConf(ZDO_StartNetworkConf_t *confirmInfo);
static void appZdpLeaveResp(ZDO_ZdpResp_t *zdpResp);
static void appApsDataIndHandler(APS_DataInd_t *ind);

static void manageBlinkingDuringRejoin(BlinkingAction_t action);
static void startingNetworkTimerFired(void);
static void identifyTimerFired(void);
static void appInitializePds(void);

#ifdef _SECURITY_
static void appInitSecurity(void);
#endif
/******************************************************************************
                              Implementations section
******************************************************************************/

/**************************************************************************//**
  \brief Application task.

  \param  None.

  \return None.
******************************************************************************/
void APL_TaskHandler(void)
{
  switch (appState)
  {
    case APP_INITING_STATE:
    {
      visualizeAppStarting();
      appInitialization();
      appState = APP_STARTING_NETWORK_STATE;
      appPostGlobalTask();
    }
    break;

    case APP_IN_NETWORK_STATE:
    {
      if (appTaskFlags.appSubTaskPosted)
      {
        appTaskFlags.appSubTaskPosted = false;

        if (deviceInterface.appDeviceTaskHandler)
          deviceInterface.appDeviceTaskHandler();
      }

      if (appTaskFlags.appMsgSenderTaskPosted)
      {
        appTaskFlags.appMsgSenderTaskPosted = false;
        appMsgSender();
      }

      if (appTaskFlags.appCmdHandlerTaskPosted)
      {
        appTaskFlags.appCmdHandlerTaskPosted = false;
        appCmdHandler();
      }
    }
    break;

    case APP_STARTING_NETWORK_STATE:
    {
      visualizeNwkStarting();
      manageBlinkingDuringRejoin(START_BLINKING);

      // Start network
      zdoStartReq.ZDO_StartNetworkConf = appZdoStartNetworkConf;
      ZDO_StartNetworkReq(&zdoStartReq);
    }
    break;

    case APP_LEAVING_NETWORK_STATE:
    {
      ZDO_MgmtLeaveReq_t *zdpLeaveReq = &leaveReq.req.reqPayload.mgmtLeaveReq;

      visualizeNwkLeaving();
      leaveReq.ZDO_ZdpResp =  appZdpLeaveResp;
      leaveReq.reqCluster = MGMT_LEAVE_CLID;
      leaveReq.dstAddrMode = EXT_ADDR_MODE;
      leaveReq.dstExtAddr = 0;
      zdpLeaveReq->deviceAddr = 0;
      zdpLeaveReq->rejoin = 0;
      zdpLeaveReq->removeChildren = 1;
      zdpLeaveReq->reserved = 0;
      ZDO_ZdpReq(&leaveReq);
    }
    break;

    default:
    break;
  }
}

/**************************************************************************//**
  \brief Main application initialization.

  \param  None.

  \return None.
******************************************************************************/
static void appInitialization(void)
{
  DeviceType_t deviceType;

#if APP_DEVICE_TYPE == DEV_TYPE_COORDINATOR
  appCoordinatorGetInterface(&deviceInterface);
#elif APP_DEVICE_TYPE == DEV_TYPE_ROUTER
  appRouterGetInterface(&deviceInterface);
#else
  appEndDeviceGetInterface(&deviceInterface);
#endif

  appInitializePds();

  deviceType = WSN_DEMO_DEVICE_TYPE;
  CS_WriteParameter(CS_DEVICE_TYPE_ID, &deviceType);
  appNwkInfo.nodeType = deviceType;

  simpleDescriptor.endpoint = WSNDEMO_ENDPOINT;
  simpleDescriptor.AppProfileId = CCPU_TO_LE16(WSNDEMO_PROFILE_ID);
  simpleDescriptor.AppDeviceId = CCPU_TO_LE16(WSNDEMO_DEVICE_ID);
  simpleDescriptor.AppDeviceVersion = WSNDEMO_DEVICE_VERSION;
  endpointParams.simpleDescriptor = &simpleDescriptor;
  endpointParams.APS_DataInd = appApsDataIndHandler;
  APS_RegisterEndpointReq(&endpointParams);

  startingNetworkTimer.interval = START_NETWORK_BLINKING_INTERVAL;
  startingNetworkTimer.mode     = TIMER_REPEAT_MODE;
  startingNetworkTimer.callback = startingNetworkTimerFired;

#ifdef _SECURITY_
  appInitSecurity();
#endif

  {
    // To remove unalignment access warning for ARM.
    ExtAddr_t extAddr;

    CS_ReadParameter(CS_UID_ID, &extAddr);
    appNwkInfo.extAddr = extAddr;
  }

#if APP_USE_OTAU == 1
  appZclManagerInit();
#endif // APP_USE_OTAU

  if (deviceInterface.appDeviceInitialization)
    deviceInterface.appDeviceInitialization();
  appInitMsgSender();
  appInitCmdHandler();
}

#ifdef _SECURITY_
/**************************************************************************//**
  \brief Security initialization.

  \param  None.

  \return None.
******************************************************************************/
static void appInitSecurity(void)
{
  uint64_t tcAddr;
#if APP_DEVICE_TYPE == DEV_TYPE_COORDINATOR
  CS_ReadParameter(CS_APS_TRUST_CENTER_ADDRESS_ID, &tcAddr);
  CS_WriteParameter(CS_UID_ID, &tcAddr);
#else
  tcAddr = CCPU_TO_LE64(COORDINATOR_EXT_ADDR);
  CS_WriteParameter(CS_APS_TRUST_CENTER_ADDRESS_ID, &tcAddr);
#endif

#ifdef _LINK_SECURITY_
  {
    uint8_t linkKey[16] = LINK_KEY;
#if APP_DEVICE_TYPE == DEV_TYPE_COORDINATOR
    ExtAddr_t extAddr = CCPU_TO_LE64(DEVICE1_EXT_ADDR);
    APS_SetLinkKey(&extAddr, linkKey);
    ExtAddr_t extAddr1 = CCPU_TO_LE64(DEVICE2_EXT_ADDR);
    APS_SetLinkKey(&extAddr1, linkKey);
#else
    ExtAddr_t extAddr = CCPU_TO_LE64(COORDINATOR_EXT_ADDR);
    APS_SetLinkKey(&extAddr, linkKey);
#endif
  }
#endif // _LINK_SECURITY_
}
#endif // _SECURITY_

/**************************************************************************//**
  \brief Initialization of PDS to protect system parameters in case of power failure.

  \param  None.

  \return None.
******************************************************************************/
static void appInitializePds(void)
{
#if defined(_COMMISSIONING_) && (APP_USE_PDS == 1)
  // Configure PDS to store and update parameters in NVM by BitCloud events
  PDS_StoreByEvents(BC_ALL_MEMORY_MEM_ID);

  // Restore required configuration from NVM, or reset NVM, if button is pressed
  BSP_OpenButtons(NULL, NULL);
  
  if (BSP_ReadButtonsState() & 0x01)
    PDS_ResetStorage(PDS_ANY_MEMORY_FROM);
  else if (PDS_IsAbleToRestore(BC_ALL_MEMORY_MEM_ID))
    PDS_Restore(BC_ALL_MEMORY_MEM_ID);

  BSP_CloseButtons();
#endif // _COMMISSIONING_ && APP_USE_PDS
}

/**************************************************************************//**
  \brief Leave response callback. The response means that the command has
         been received successfully but not precessed yet.

  \param[in]  zdpResp - ZDP response.

  \return None.
******************************************************************************/
static void appZdpLeaveResp(ZDO_ZdpResp_t *zdpResp)
{
  if (ZDO_SUCCESS_STATUS == zdpResp->respPayload.status)
    appState = APP_STOP_STATE;
  else
    appPostGlobalTask();
}

/**************************************************************************//**
  \brief ZDO_StartNetwork primitive confirmation was received.

  \param[in]  startInfo - confirmation information.

  \return None.
******************************************************************************/
static void appZdoStartNetworkConf(ZDO_StartNetworkConf_t *startInfo)
{
  manageBlinkingDuringRejoin(STOP_BLINKING);

  if (ZDO_SUCCESS_STATUS == startInfo->status)
  {
    appState = APP_IN_NETWORK_STATE;
    visualizeNwkStarted();
#if APP_USE_OTAU == 1
    runOtauService();
#endif // APP_USE_OTAU
    if (deviceInterface.appDeviceTaskReset)
      deviceInterface.appDeviceTaskReset();

    // Network parameters, such as short address, should be saved
    appNwkInfo.panID           = startInfo->PANId;
    appNwkInfo.shortAddr       = startInfo->shortAddr;
    appNwkInfo.parentShortAddr = startInfo->parentAddr;
    appNwkInfo.workingChannel  = startInfo->activeChannel;
  }
  else
    appPostGlobalTask();
}

/**************************************************************************//**
  \brief Indicates network parameters update.

  \param[in]  nwkParams - information about network updates.

  \return None.
******************************************************************************/
void ZDO_MgmtNwkUpdateNotf(ZDO_MgmtNwkUpdateNotf_t *nwkParams)
{
  if (ZDO_NO_KEY_PAIR_DESCRIPTOR_STATUS == nwkParams->status)
  {
#ifdef _LINK_SECURITY_
    ExtAddr_t addr        = nwkParams->childInfo.extAddr;
    uint8_t   linkKey[16] = LINK_KEY;
    APS_SetLinkKey(&addr, linkKey);
#endif
  }
  else
    appZdoNwkUpdateHandler(nwkParams);
}

/**************************************************************************//**
  \brief Application handling of network parameters update event.

  \param[in]  updateParam - network parameters update notification.

  \return None.
******************************************************************************/
static void appZdoNwkUpdateHandler(ZDO_MgmtNwkUpdateNotf_t *updateParam)
{
  switch (updateParam->status)
  {
    case ZDO_NETWORK_LOST_STATUS:
      appState = APP_STOP_STATE;
      break;

    case ZDO_NETWORK_LEFT_STATUS:
      visualizeNwkLeft();
      appState = APP_STARTING_NETWORK_STATE;
      appPostGlobalTask();
      break;

    case ZDO_NWK_UPDATE_STATUS:
    case ZDO_NETWORK_STARTED_STATUS:
      appNwkInfo.shortAddr       = updateParam->nwkUpdateInf.shortAddr;
      appNwkInfo.panID           = updateParam->nwkUpdateInf.panId;
      appNwkInfo.parentShortAddr = updateParam->nwkUpdateInf.parentShortAddr;
      appNwkInfo.workingChannel  = updateParam->nwkUpdateInf.currentChannel;

      manageBlinkingDuringRejoin(STOP_BLINKING);
      visualizeNwkStarted();

      if (APP_STARTING_NETWORK_STATE == appState)
      {
#if APP_USE_OTAU == 1
        runOtauService();
#endif // APP_USE_OTAU
        if (deviceInterface.appDeviceTaskReset)
          deviceInterface.appDeviceTaskReset();
      }
      if (APP_STOP_STATE == appState)
      {
        appState = APP_IN_NETWORK_STATE;
        if (deviceInterface.appDeviceTaskReset)
          deviceInterface.appDeviceTaskReset();
        appPostGlobalTask();
      }
      appState = APP_IN_NETWORK_STATE;

      break;

    default:
      break;
  }
}

/**************************************************************************//**
  \brief Sleep timer wake up indication.

  \param  None.

  \return None.
******************************************************************************/
void ZDO_WakeUpInd(void)
{
  if (deviceInterface.appDeviceWakeUpInd)
    deviceInterface.appDeviceWakeUpInd();
}

/**************************************************************************//**
  \brief Blinking during rejoin control.

  \param[in]  action - blinking action.

  \return None.
******************************************************************************/
static void manageBlinkingDuringRejoin(BlinkingAction_t action)
{
  static bool run = false;

  if (START_BLINKING == action)
  {
    if (!run)
    {
      HAL_StartAppTimer(&startingNetworkTimer);
      run = true;
    }
  }

  if (STOP_BLINKING == action)
  {
    run = false;
    HAL_StopAppTimer(&startingNetworkTimer);
  }
}

/**************************************************************************//**
  \brief Network starting timer event.

  \param  None.

  \return None.
******************************************************************************/
static void startingNetworkTimerFired(void)
{
  visualizeNwkStarting();
}
/**************************************************************************//**
  \brief Reads LQI and RSSI for a parent node into appNwkInfo structure.

  \param  None.

  \return None.
******************************************************************************/
void appReadLqiRssi(void)
{
  ZDO_GetLqiRssi_t lqiRssi;

  lqiRssi.nodeAddr = appNwkInfo.parentShortAddr;
  ZDO_GetLqiRssi(&lqiRssi);

  appNwkInfo.lqi  = lqiRssi.lqi;
  appNwkInfo.rssi = lqiRssi.rssi;
}
/**************************************************************************//**
  \brief Posts application main task.

  \param  None.

  \return None.
******************************************************************************/
static void appPostGlobalTask(void)
{
  SYS_PostTask(APL_TASK_ID);
}

/**************************************************************************//**
  \brief Posts application sub task.

  \param  None.

  \return None.
******************************************************************************/
void appPostSubTaskTask(void)
{
  if (APP_IN_NETWORK_STATE == appState)
  {
    appTaskFlags.appSubTaskPosted = true;
    SYS_PostTask(APL_TASK_ID);
  }
}

/**************************************************************************//**
  \brief Posts command handler task.

  \param  None.

  \return None.
******************************************************************************/
void appPostCmdHandlerTask(void)
{
  if (APP_IN_NETWORK_STATE == appState)
  {
    appTaskFlags.appCmdHandlerTaskPosted = true;
    SYS_PostTask(APL_TASK_ID);
  }
}

/**************************************************************************//**
  \brief Posts message sender task.

  \param  None.

  \return None.
******************************************************************************/
void appPostMsgSenderTask(void)
{
  if (APP_IN_NETWORK_STATE == appState)
  {
    appTaskFlags.appMsgSenderTaskPosted = true;
    SYS_PostTask(APL_TASK_ID);
  }
}

/**************************************************************************//**
  \brief Initiate leave network procedure.

  \param  None.

  \return None.
******************************************************************************/
void appLeaveNetwork(void)
{
  if (APP_IN_NETWORK_STATE == appState)
  {
    appState = APP_LEAVING_NETWORK_STATE;
    appPostGlobalTask();
  }
}

/**************************************************************************//**
  \brief Returs sleep is enabled flag.

  \param  None.

  \return true if sleep is enabled, false otherwise.
******************************************************************************/
bool appIsSleepPermitted(void)
{
  return (!identifyDuration &&
          appIsCmdQueueFree() &&
          appIsTxQueueFree());
}

/**************************************************************************//**
  \brief New command received from PC event handler.

  \param[in] command - pointer to received command.
  \param[in] cmdSize - received command size.

  \return None.
******************************************************************************/
void appPcCmdReceived(void *command, uint8_t cmdSize)
{
  AppCommand_t *pCommand = (AppCommand_t *)command;

  appCreateCommand(&pCommand);

  // Warning prevention
  (void)cmdSize;
}

/**************************************************************************//**
  \brief Starts blinking for identifying device.

  \param[in] blinkDuration - blinking duration.
  \param[in] blinkPeriod - blinking period.

  \return none
******************************************************************************/
void appStartIdentifyVisualization(uint16_t blinkDuration, uint16_t blinkPeriod)
{
  if (blinkDuration && blinkPeriod)
  {
    identifyDuration = blinkDuration;

    HAL_StopAppTimer(&identifyTimer); // Have to be stopped before start
    identifyTimer.interval = blinkPeriod;
    identifyTimer.mode     = TIMER_REPEAT_MODE;
    identifyTimer.callback = identifyTimerFired;
    HAL_StartAppTimer(&identifyTimer);
  }
}

/**************************************************************************//**
  \brief Identify timer event.

  \param  None.

  \return None.
******************************************************************************/
static void identifyTimerFired(void)
{
  visualizeIdentity();

  identifyDuration -= MIN(identifyDuration, identifyTimer.interval);

  if (!identifyDuration)
  {
    visualizeWakeUp();
    HAL_StopAppTimer(&identifyTimer);
  }
}

/**************************************************************************//**
  \brief Finds command descriptor in appCmdDescTable.

  \param[in, out] pCmdDesc - pointer to command descriptor
  \param[in] cmdId - command identificator.

  \return true if search has been successful, false otherwise.
******************************************************************************/
bool appGetCmdDescriptor(AppCommandDescriptor_t *pCmdDesc, uint8_t cmdId)
{
  if (deviceInterface.appDeviceCmdDescTable)
  {
    for (uint8_t i = 0; i < deviceInterface.appDeviceCmdDescTableSize; i++)
    {
      memcpy_P(pCmdDesc, &deviceInterface.appDeviceCmdDescTable[i], sizeof(AppCommandDescriptor_t));

      if (cmdId == pCmdDesc->commandID)
      {
        return true;
      }
    }
  }

  memset(pCmdDesc, 0, sizeof(AppCommandDescriptor_t));
  return false;
}

/**************************************************************************//**
  \brief APS data indication handler.

  \param ind - received data.

  \return none.
******************************************************************************/
static void appApsDataIndHandler(APS_DataInd_t *ind)
{
  AppCommand_t *pCommand = (AppCommand_t *)ind->asdu;

  visualizeAirRxFinished();
  appCreateCommand(&pCommand);
}

#ifdef _BINDING_
/**************************************************************************//**
  \brief Stub for ZDO Binding Indication.

  \param bindInd - indication.

  \return none.
******************************************************************************/
void ZDO_BindIndication(ZDO_BindInd_t *bindInd)
{
  (void)bindInd;
}

/**************************************************************************//**
  \brief Stub for ZDO Unbinding Indication.

  \param unbindInd - indication.

  \return none.
******************************************************************************/
void ZDO_UnbindIndication(ZDO_UnbindInd_t *unbindInd)
{
  (void)unbindInd;
}
#endif //_BINDING_

/**********************************************************************//**
  \brief Main - C program main start function

  \param none

  \return none
**************************************************************************/
int main(void)
{ 
  SYS_SysInit();

  for (;;)
  {
    SYS_RunTask();
  }
}
//eof WSNDemoApp.c

