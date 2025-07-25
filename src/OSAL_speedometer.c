#include "hal_types.h"
#include "OSAL.h"
#include "OSAL_Tasks.h"

/* HAL */
#include "hal_drivers.h"
#include "hal_assert.h"

/* LL */
#include "ll.h"

/* HCI */
#include "hci_tl.h"

#ifdef OSAL_CBTIMER_NUM_TASKS
#include "osal_cbtimer.h"
#endif

/* L2CAP */
#include "l2cap.h"

/* gap */
#include "gap.h"
#include "gapgattserver.h"
#include "gapbondmgr.h"

/* GATT */
#include "gatt.h"

#include "gattservapp.h"

/* Profiles */
#include "peripheral.h"

/* Application */
#include "speedometer.h"

/*********************************************************************
 * GLOBAL VARIABLES
 */

 // The order in this table must be identical to the task initialization calls below in osalInitTask.
const pTaskEventHandlerFn tasksArr[] =
{
  LL_ProcessEvent,                                                  // task 0
  Hal_ProcessEvent,                                                 // task 1
  HCI_ProcessEvent,                                                 // task 2
#ifdef OSAL_CBTIMER_NUM_TASKS
  OSAL_CBTIMER_PROCESS_EVENT(osal_CbTimerProcessEvent),             // task 3
#endif
  L2CAP_ProcessEvent,                                               // task 4
  GAP_ProcessEvent,                                                 // task 5
  SM_ProcessEvent,                                                  // task 6
  GATT_ProcessEvent,                                                // task 7
  GAPRole_ProcessEvent,                                             // task 8
  GAPBondMgr_ProcessEvent,                                          // task 9
  GATTServApp_ProcessEvent,                                         // task 10
  Speedometer_ProcessEvent                                          // task 11
};

const uint8 tasksCnt = sizeof(tasksArr) / sizeof(tasksArr[0]);
uint16* tasksEvents;

/*********************************************************************
 * FUNCTIONS
 *********************************************************************/

 /*********************************************************************
  * @fn      osalInitTasks
  *
  * @brief   This function invokes the initialization function for each task.
  *
  * @param   void
  *
  * @return  none
  */
void osalInitTasks(void)
{
    uint8 taskID = 0;

    tasksEvents = (uint16*)osal_mem_alloc(sizeof(uint16) * tasksCnt);

    /* The tasksEvents allocated pointer must be valid */
    if (tasksEvents != NULL)
    {
        osal_memset(tasksEvents, 0, (sizeof(uint16) * tasksCnt));
    }
    else
    {
        HAL_ASSERT_FORCED();
    }

    /* LL Task */
    LL_Init(taskID++);

    /* Hal Task */
    Hal_Init(taskID++);

    /* HCI Task */
    HCI_Init(taskID++);

#if defined ( OSAL_CBTIMER_NUM_TASKS )
    /* Callback Timer Tasks */
    osal_CbTimerInit(taskID);
    taskID += OSAL_CBTIMER_NUM_TASKS;
#endif

    /* L2CAP Task */
    L2CAP_Init(taskID++);

    /* GAP Task */
    GAP_Init(taskID++);

    /* SM Task */
    SM_Init(taskID++);

    /* GATT Task */
    GATT_Init(taskID++);

    /* Profiles */
    GAPRole_Init(taskID++);
    GAPBondMgr_Init(taskID++);

    GATTServApp_Init(taskID++);

    /* Application */
    Speedometer_Init(taskID);
}