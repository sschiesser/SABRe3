/**********************************************************************//**
  \file WSNCommandHandler.c

  \brief Contains functions for buffering and sequenced handling of
         application commands.

  \author
    Atmel Corporation: http://www.atmel.com \n
    Support email: avr@atmel.com

  Copyright (c) 2008-2012, Atmel Corporation. All rights reserved.
  Licensed under Atmel's Limited License Agreement (BitCloudTM).

  \internal
  History:
    24/01/12 D. Kolmakov - Created

  Last change:
    $Id: WSNCommandHandler.c 21155 2012-05-11 13:57:07Z nfomin $
**************************************************************************/

/*****************************************************************************
                              Includes section
******************************************************************************/
#include <mnUtils.h>
#include <queue.h>
#include <WSNDemoApp.h>
#include <WSNCommandHandler.h>
#include <WSNVisualizer.h>

/*****************************************************************************
                              Types section
******************************************************************************/
typedef struct _AppCmdQueueElement_t
{
  /** Service field, used for queue support */
  QueueElement_t next;

  /** Application command */
  AppCommand_t command;

} AppCmdQueueElement_t;

/****************************************************************************
                              Static functions prototypes section
******************************************************************************/
static AppCommand_t *appGetCommandToProcess(void);

/*****************************************************************************
                              Local variables section
******************************************************************************/

/** Buffers for application commands */
static AppCmdQueueElement_t  appCmdBuffers[MAX_CMD_QUEUE_COUNT];

/** Queue of incoming application commands. */
static DECLARE_QUEUE(appBusyCmdQueue);

/** Queue of free cells for commands. */
static DECLARE_QUEUE(appFreeCmdQueue);

/******************************************************************************
                              Implementations section
******************************************************************************/
/**************************************************************************//**
  \brief Initializes command handler.

  \param  None.

  \return None.
******************************************************************************/
void appInitCmdHandler(void)
{
  resetQueue(&appBusyCmdQueue);
  resetQueue(&appFreeCmdQueue);

  for (uint8_t i = 0; i < ARRAY_SIZE(appCmdBuffers); i++)
  {
    putQueueElem(&appFreeCmdQueue, &appCmdBuffers[i].next);
  }
}

/**************************************************************************//**
  \brief Handles application command from appCmdQueue.

  \param  None.

  \return None.
******************************************************************************/
void appCmdHandler(void)
{
  AppCommand_t *currentCommand = appGetCommandToProcess();

  if (currentCommand)
  {
    bool deleteCmd = true;
    AppCommandDescriptor_t cmdDesc;

    if (appGetCmdDescriptor(&cmdDesc, currentCommand->id))
    {
      deleteCmd = cmdDesc.serviceVector(currentCommand);
    }

    if (deleteCmd)
    {
      putQueueElem(&appFreeCmdQueue, deleteHeadQueueElem(&appBusyCmdQueue));
    }
    else
    {
      putQueueElem(&appBusyCmdQueue, deleteHeadQueueElem(&appBusyCmdQueue));
    }

    appPostCmdHandlerTask();
  }
}

/**************************************************************************//**
  \brief Returns current command queue status.

  \param  None.

  \return true if command queue is free, false otherwise.
******************************************************************************/
bool appIsCmdQueueFree(void)
{
  return appGetCommandToProcess() ? false : true;
}

/**************************************************************************//**
  \brief Returns pointer to first element in command queue.

  \return pointer to AppCommand_t if queue is not empty, NULL otherwise.
******************************************************************************/
static AppCommand_t *appGetCommandToProcess(void)
{
  QueueElement_t *queueElement = getQueueElem(&appBusyCmdQueue);

  if (queueElement)
  {
    return &(GET_STRUCT_BY_FIELD_POINTER(AppCmdQueueElement_t,
                                         next,
                                         queueElement))->command;
  }

  return NULL;
}

/**************************************************************************//**
  \brief Inserts new element into the command queue.

  \param[in,out] pCommandPtr - pointer to pointer to AppCommand_t. On the output
                               if function returns true *pCommandPtr will refer
                               to allocated cell.

  \return true if insertion was successful, false otherwise.
******************************************************************************/
bool appCreateCommand(AppCommand_t **pCommandPtr)
{
  AppCmdQueueElement_t *appCmdQueueElement = NULL;
  QueueElement_t *freeElement = NULL;
  bool result = false;

  assert(pCommandPtr, CMDQUEUEINSERTASSERT_0);

  freeElement = getQueueElem(&appFreeCmdQueue);

  if (freeElement)
  {
    putQueueElem(&appBusyCmdQueue, deleteHeadQueueElem(&appFreeCmdQueue));
    appCmdQueueElement = GET_STRUCT_BY_FIELD_POINTER(AppCmdQueueElement_t,
                                                     next,
                                                     freeElement);

    if (*pCommandPtr)
    {
      memcpy(&appCmdQueueElement->command, *pCommandPtr, sizeof(AppCommand_t));
    }

    *pCommandPtr = &appCmdQueueElement->command;

    result = true;
  }

  appPostCmdHandlerTask();
  return result;
}

