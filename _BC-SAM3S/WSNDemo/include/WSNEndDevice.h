/***************************************************************************//**
  \file WSNEndDevice.h

  \brief Contains function prototypes related to end device functionality.

  \author
    Atmel Corporation: http://www.atmel.com \n
    Support email: avr@atmel.com

  Copyright (c) 2008-2012, Atmel Corporation. All rights reserved.
  Licensed under Atmel's Limited License Agreement (BitCloudTM).

  \internal
  History:
    15/03/12 D. Kolmakov - Created

  Last change:
    $Id: WSNEndDevice.h 20608 2012-03-23 09:22:15Z arazinkov $
*******************************************************************************/
#ifndef _WSNENDDEVICE_H
#define _WSNENDDEVICE_H

/*****************************************************************************
                              Includes section
******************************************************************************/
#include <WSNDemoApp.h>

/*****************************************************************************
                              Prototypes section
******************************************************************************/
/**************************************************************************//**
  \brief Fills device interface structure with functions related to coordinator.

  \param[in, out] deviceInterface - pointer to device interface structure.

  \return None.
******************************************************************************/
void appEndDeviceGetInterface(SpecialDeviceInterface_t *deviceInterface);

#endif // _WSNENDDEVICE_H
