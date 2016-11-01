/**********************************************************************//**
  \file WSNSensorManager.c

  \brief

  \author
    Atmel Corporation: http://www.atmel.com \n
    Support email: avr@atmel.com

  Copyright (c) 2008-2012, Atmel Corporation. All rights reserved.
  Licensed under Atmel's Limited License Agreement (BitCloudTM).

  \internal
  History:
    13/06/07 I. Kalganova - Modified

  Last change:
    $Id: WSNSensorManager.c 20608 2012-03-23 09:22:15Z arazinkov $
**************************************************************************/

/*****************************************************************************
                              Includes section
******************************************************************************/
#include <types.h>
#include <taskManager.h>
#include <sensors.h>
#include <WSNDemoApp.h>
#include <WSNVisualizer.h>

/*****************************************************************************
                              External variables section
******************************************************************************/
extern AppNwkInfoCmdPayload_t appNwkInfo;

/*****************************************************************************
                              Local variables section
******************************************************************************/
static void (*callback)(void);

/****************************************************************************
                              Static functions prototypes section
******************************************************************************/
static void temperatureDataReady(bool result, int16_t temperature);
static void lightDataReady(bool result, int16_t light);
static void batteryDataReady(int16_t battery);

/******************************************************************************
                              Implementations section
******************************************************************************/
/**************************************************************************//**
  \brief Sensors initialization.

  \param None.

  \return None
******************************************************************************/
void appStartSensorManager(void)
{
  assert(BC_SUCCESS == BSP_OpenTemperatureSensor(), BSPRESDSENSORDATAASSERT_6);
  assert(BC_SUCCESS == BSP_OpenLightSensor(), BSPRESDSENSORDATAASSERT_7);
  assert(BC_SUCCESS == BSP_OpenBatterySensor(), BSPRESDSENSORDATAASSERT_8);
}

/**************************************************************************//**
  \brief Closes all sensors.

  \param None.

  \return None
******************************************************************************/
void appStopSensorManager()
{
  assert(BC_SUCCESS == BSP_CloseTemperatureSensor(), BSPRESDSENSORDATAASSERT_3);
  assert(BC_SUCCESS == BSP_CloseLightSensor(), BSPRESDSENSORDATAASSERT_4);
  assert(BC_SUCCESS == BSP_CloseBatterySensor(), BSPRESDSENSORDATAASSERT_5);
}

/**************************************************************************//**
  \brief Sensors data request.

  \param[in] sensorsGot - on sensors reading finished callback.

  \return None.
******************************************************************************/
void appGetSensorData(void (*sensorsGot)(void))
{
  callback = sensorsGot;
  assert(BC_SUCCESS == BSP_ReadTemperatureData(temperatureDataReady), BSPRESDSENSORDATAASSERT_0);
}

/**************************************************************************//**
  \brief Temperature reading done callback.

  \param[in] result - successful temperature reading flag.
  \param[in] temperature - result of temperature reading, if result is true.

  \return None.
******************************************************************************/
static void temperatureDataReady(bool result, int16_t temperature)
{
  if (result)
    appNwkInfo.boardInfo.meshbean.temperature = CPU_TO_LE32(temperature);
  else
    appNwkInfo.boardInfo.meshbean.temperature = CPU_TO_LE32(0);

  assert(BC_SUCCESS == BSP_ReadLightData(lightDataReady), BSPRESDSENSORDATAASSERT_1);
}

/**************************************************************************//**
  \brief Illumination reading done callback.

  \param[in] result - successful illumination reading flag.
  \param[in] light - result of illumination reading, if result is true.

  \return None.
******************************************************************************/
static void lightDataReady(bool result, int16_t light)
{
  if (result)
    appNwkInfo.boardInfo.meshbean.light = CPU_TO_LE32(light);
  else
    appNwkInfo.boardInfo.meshbean.light = CPU_TO_LE32(0);

  assert(BC_SUCCESS == BSP_ReadBatteryData(batteryDataReady), BSPRESDSENSORDATAASSERT_2);
}

/**************************************************************************//**
  \brief Battery level reading done callback.

  \param[in] result - successful battery level reading flag.
  \param[in] light - result of battery level reading, if result is true.

  \return None.
******************************************************************************/
static void batteryDataReady(int16_t battery)
{
  appNwkInfo.boardInfo.meshbean.battery = CPU_TO_LE32(battery);
  callback();
}

//eof WSNSensorManager.c
