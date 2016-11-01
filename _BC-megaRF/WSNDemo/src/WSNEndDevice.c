/**********************************************************************//**
  \file WSNEndDevice.c

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
    $Id: WSNEndDevice.c 20719 2012-03-30 16:27:17Z dkolmakov $
**************************************************************************/
#ifdef _ENDDEVICE_

/*****************************************************************************
                              Includes section
******************************************************************************/
#include <WSNDemoApp.h>
#include <WSNVisualizer.h>
#include <WSNCommandHandler.h>
#include <WSNMessageSender.h>
#include <WSNCommand.h>

/*****************************************************************************
                              Prototypes section
******************************************************************************/
bool appEndDeviceIdentifyCmdHandler(AppCommand_t *pCommand);
bool appEndDeviceNwkInfoCmdHandler(AppCommand_t *pCommand);

/******************************************************************************
                              Constants section
******************************************************************************/
/**//**
 * \brief Command descriptor table. Is used by WSNCommandHandler.
 */
PROGMEM_DECLARE(AppCommandDescriptor_t appEndDeviceCmdDescTable[]) =
{
  APP_COMMAND_DESCRIPTOR(APP_NETWORK_INFO_COMMAND_ID, appEndDeviceNwkInfoCmdHandler),
  APP_COMMAND_DESCRIPTOR(APP_IDENTIFY_COMMAND_ID, appEndDeviceIdentifyCmdHandler)
};

/*****************************************************************************
                              External variables section
******************************************************************************/
extern AppNwkInfoCmdPayload_t appNwkInfo;
extern AppState_t appState;

/****************************************************************************
                              Static functions prototypes section
******************************************************************************/
static void appEndDeviceSleepConf(ZDO_SleepConf_t *conf);
static void appSensorsGot(void);
static void deviceTimerFired(void);
static void appEndDeviceTaskHandler(void);
static void appEndDeviceInitialization(void);
static void appEndDeviceTaskReset(void);
static void appEndDeviceSleepConf(ZDO_SleepConf_t *conf);
static void appEndDeviceWakeUpInd(void);

/*****************************************************************************
                              Local variables section
******************************************************************************/
static ZDO_SleepReq_t sleepReq;
static DeviceState_t  appDeviceState = INITIAL_DEVICE_STATE;
static HAL_AppTimer_t deviceTimer;

/******************************************************************************
                              Implementations section
******************************************************************************/
/**************************************************************************//**
  \brief End device application subtask handler.

  \param  None.

  \return None.
******************************************************************************/
static void appEndDeviceTaskHandler(void)
{
  switch (appDeviceState)
  {
    case WAITING_DEVICE_STATE:
    {
      if (appIsSleepPermitted())
      {
        appDeviceState = SLEEPPING_DEVICE_STATE;
      }

      appPostSubTaskTask();
    }
    break;

    case READING_SENSORS_STATE:
    {
      appReadLqiRssi();
      appStartSensorManager();
      appDeviceState = WAITING_DEVICE_STATE;
      appGetSensorData(appSensorsGot);
    }
    break;

    case SENDING_DEVICE_STATE:
    {
      AppCommand_t *pCommand = NULL;

      if (appCreateCommand(&pCommand))
      {
        pCommand->id = APP_NETWORK_INFO_COMMAND_ID;
        memcpy(&pCommand->payload.nwkInfo, &appNwkInfo, sizeof(AppNwkInfoCmdPayload_t));
      }

      appDeviceState = STARTING_TIMER_STATE;
      appPostSubTaskTask();
    }
    break;

    case STARTING_TIMER_STATE:
    {
      HAL_StartAppTimer(&deviceTimer);
    }
    break;

    case SLEEPPING_DEVICE_STATE:
    {
      visualizeSleep();
      appDeviceState = WAITING_DEVICE_STATE;
      ZDO_SleepReq(&sleepReq);
    }
    break;

    case INITIAL_DEVICE_STATE:
    {
      HAL_StopAppTimer(&deviceTimer);
      deviceTimer.interval = APP_TIMER_END_DEVICE_MIN_WAKEFUL_TIME;
      deviceTimer.mode     = TIMER_ONE_SHOT_MODE;
      deviceTimer.callback = deviceTimerFired;

      sleepReq.ZDO_SleepConf = appEndDeviceSleepConf;
      appDeviceState = READING_SENSORS_STATE;
      appPostSubTaskTask();
    }
    break;

    default:
    break;
  }
}

/**************************************************************************//**
  \brief Initializes end device application subtask parameters.

  \param  None.

  \return None.
******************************************************************************/
static void appEndDeviceInitialization(void)
{
  bool rxOnWhenIdleFlag = false;

  CS_WriteParameter(CS_RX_ON_WHEN_IDLE_ID, &rxOnWhenIdleFlag);
  appDeviceState = INITIAL_DEVICE_STATE;
}

/**************************************************************************//**
  \brief Resets end device subtask.

  \param  None.

  \return None.
******************************************************************************/
static void appEndDeviceTaskReset(void)
{
  appDeviceState = INITIAL_DEVICE_STATE;
  appPostSubTaskTask();
}
/**************************************************************************//**
  \brief Sensors reading done callback.

  \param  None.

  \return None.
******************************************************************************/
static void appSensorsGot(void)
{
  appStopSensorManager();
  appDeviceState = SENDING_DEVICE_STATE;
  appPostSubTaskTask();
}

/**************************************************************************//**
  \brief Sleep request confirm.

  \param[in]  conf - parameters of the confirm.

  \return None.
******************************************************************************/
static void appEndDeviceSleepConf(ZDO_SleepConf_t *conf)
{
  if (ZDO_SUCCESS_STATUS == conf->status)
  {
    appDeviceState = SLEEPPING_DEVICE_STATE;
  }
  else
  {
    appDeviceState = SLEEPPING_DEVICE_STATE;
    appPostSubTaskTask();
  }
}

/**************************************************************************//**
  \brief Wake up indication handling.

  \param  None.

  \return None.
******************************************************************************/
static void appEndDeviceWakeUpInd(void)
{
  visualizeWakeUp();
  appDeviceState = READING_SENSORS_STATE;
  appPostSubTaskTask();
}

/**************************************************************************//**
  \brief Device timer event.

  \param  None.

  \return None.
******************************************************************************/
static void deviceTimerFired(void)
{
  if (appIsSleepPermitted())
    appDeviceState = SLEEPPING_DEVICE_STATE;
  else
    appDeviceState = STARTING_TIMER_STATE;

  appPostSubTaskTask();
}

/**************************************************************************//**
  \brief Application network info command handler.

  \param[in] pCommand - pointer to application command.

  \return true if deletion is needed, false otherwise.
******************************************************************************/
bool appEndDeviceNwkInfoCmdHandler(AppCommand_t *pCommand)
{
  APS_DataReq_t *pMsgParams = NULL;

  if (appCreateTxFrame(&pMsgParams, &pCommand, NULL))
  {
    memset(pMsgParams, 0, sizeof(APS_DataReq_t));

    pMsgParams->profileId               = CCPU_TO_LE16(WSNDEMO_PROFILE_ID);
    pMsgParams->dstAddrMode             = APS_SHORT_ADDRESS;
    pMsgParams->dstAddress.shortAddress = CPU_TO_LE16(0);
    pMsgParams->dstEndpoint             = 1;
    pMsgParams->clusterId               = CPU_TO_LE16(1);
    pMsgParams->srcEndpoint             = WSNDEMO_ENDPOINT;
    pMsgParams->asduLength              = sizeof(AppNwkInfoCmdPayload_t) + sizeof(pCommand->id);
    pMsgParams->txOptions.acknowledgedTransmission = 1;
#ifdef _APS_FRAGMENTATION_
    pMsgParams->txOptions.fragmentationPermitted = 1;
#endif
#ifdef _LINK_SECURITY_
    pMsgParams->txOptions.securityEnabledTransmission = 1;
#endif
    pMsgParams->radius                  = 0x0;
  }

  return true;
}

/**************************************************************************//**
  \brief Application identify command handler.

  \param[in] pCommand - pointer to application command.

  \return true if deletion is needed, false otherwise.
******************************************************************************/
bool appEndDeviceIdentifyCmdHandler(AppCommand_t *pCommand)
{
  appStartIdentifyVisualization(pCommand->payload.identify.blinkDurationMs,
                                pCommand->payload.identify.blinkPeriodMs);

  return true;
}

/**************************************************************************//**
  \brief Fills device interface structure with functions related to coordinator.

  \param[in, out] deviceInterface - pointer to device interface structure.

  \return None.
******************************************************************************/
void appEndDeviceGetInterface(SpecialDeviceInterface_t *deviceInterface)
{
  deviceInterface->appDeviceCmdDescTable = appEndDeviceCmdDescTable;
  deviceInterface->appDeviceCmdDescTableSize = sizeof(appEndDeviceCmdDescTable);
  deviceInterface->appDeviceInitialization = appEndDeviceInitialization;
  deviceInterface->appDeviceTaskHandler = appEndDeviceTaskHandler;
  deviceInterface->appDeviceTaskReset = appEndDeviceTaskReset;
  deviceInterface->appDeviceWakeUpInd = appEndDeviceWakeUpInd;
}

#endif
//eof WSNEndDevice.c
