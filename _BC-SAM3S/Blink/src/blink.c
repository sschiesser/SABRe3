/**************************************************************************//**
  \file blink.c

  \brief Blink application.

  \author
    Atmel Corporation: http://www.atmel.com \n
    Support email: avr@atmel.com

  Copyright (c) 2008-2012, Atmel Corporation. All rights reserved.
  Licensed under Atmel's Limited License Agreement (BitCloudTM).

  \internal
    History:
******************************************************************************/

#include <appTimer.h>
#include <zdo.h>
#include <blink.h>
#include <taskManager.h>

static HAL_AppTimer_t blinkTimer;                           // Blink timer.

static HAL_AppTimer_t changeBlinkTimer;                     // Buttons emulation timer.

static void buttonsReleased(uint8_t buttonNumber);          // Button release event handler.
static void blinkTimerFired(void);                          // blinkTimer handler.

static void changeTimerFired(void);                         //Buttons emulation timer handler.

/*******************************************************************************
  Description: application task handler.

  Parameters: none.

  Returns: nothing.
*******************************************************************************/
void APL_TaskHandler(void)
{
  BSP_OpenLeds(); // Enable LEDs

  BSP_OnLed(LED_RED);
  BSP_OnLed(LED_YELLOW);
  BSP_OnLed(LED_GREEN);

  BSP_OpenButtons(NULL, buttonsReleased);         // Register button event handlers
  // Configure blink timer
  changeBlinkTimer.interval = 10000;              // Timer interval
  changeBlinkTimer.mode     = TIMER_REPEAT_MODE;  // Repeating mode (TIMER_REPEAT_MODE or TIMER_ONE_SHOT_MODE)
  changeBlinkTimer.callback = changeTimerFired;   // Callback function for timer fire event
  HAL_StartAppTimer(&changeBlinkTimer);           // Start blink timer      */
  // Configure blink timer
  blinkTimer.interval = APP_BLINK_INTERVAL;       // Timer interval
  blinkTimer.mode     = TIMER_REPEAT_MODE;        // Repeating mode (TIMER_REPEAT_MODE or TIMER_ONE_SHOT_MODE)
  blinkTimer.callback = blinkTimerFired;          // Callback function for timer fire event
  HAL_StartAppTimer(&blinkTimer);                 // Start blink timer
}

void changeTimerFired(void)
{
  static uint8_t button = APP_HALF_PERIOD_BUTTON;
  //Buttons emulation
  buttonsReleased(button);
  if (APP_HALF_PERIOD_BUTTON == button)
    button = APP_DOUBLE_PERIOD_BUTTON;
  else
    button = APP_HALF_PERIOD_BUTTON;
}
/*******************************************************************************
  Description: blinking timer fire event handler.

  Parameters: none.

  Returns: nothing.
*******************************************************************************/
static void blinkTimerFired()
{
  BSP_ToggleLed(LED_RED);
  BSP_ToggleLed(LED_YELLOW);
  BSP_ToggleLed(LED_GREEN);
}

/*******************************************************************************
  Description: button release event handler.

  Parameters: buttonNumber - released button number.

  Returns: nothing.
*******************************************************************************/
static void buttonsReleased(uint8_t buttonNumber)
{
  HAL_StopAppTimer(&blinkTimer); // Stop blink timer

  // Dependent on button being released, update blink interval
  if (APP_HALF_PERIOD_BUTTON == buttonNumber)
  {
    blinkTimer.interval /= 2;
    if (blinkTimer.interval < APP_MIN_BLINK_INTERVAL)
      blinkTimer.interval = APP_MIN_BLINK_INTERVAL;
  }
  else if (APP_DOUBLE_PERIOD_BUTTON == buttonNumber)
  {
    blinkTimer.interval *= 2;
    if (blinkTimer.interval > APP_MAX_BLINK_INTERVAL)
      blinkTimer.interval = APP_MAX_BLINK_INTERVAL;
  }

  blinkTimerFired();              // Update LED status immediately.
  HAL_StartAppTimer(&blinkTimer); // Start updated blink timer.
}

/*******************************************************************************
  Description: just a stub.

  Parameters: are not used.

  Returns: nothing.
*******************************************************************************/
void ZDO_MgmtNwkUpdateNotf(ZDO_MgmtNwkUpdateNotf_t *nwkParams)
{
  nwkParams = nwkParams;  // Unused parameter warning prevention
}

/*******************************************************************************
  Description: just a stub.

  Parameters: none.

  Returns: nothing.
*******************************************************************************/
void ZDO_WakeUpInd(void)
{
}

#ifdef _BINDING_
/***********************************************************************************
  Stub for ZDO Binding Indication

  Parameters:
    bindInd - indication

  Return:
    none

 ***********************************************************************************/
void ZDO_BindIndication(ZDO_BindInd_t *bindInd)
{
  (void)bindInd;
}

/***********************************************************************************
  Stub for ZDO Unbinding Indication

  Parameters:
    unbindInd - indication

  Return:
    none

 ***********************************************************************************/
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

  for(;;)
  {
    SYS_RunTask();
  }
}

//eof blink.c
