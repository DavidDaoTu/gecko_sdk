/***************************************************************************//**
 * @file sl_wfx_secure_link_task.c
 * @brief WFX FMAC driver host Secure Link task
 *******************************************************************************
 * # License
 * <b>Copyright 2022 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * The licensor of this software is Silicon Laboratories Inc.  Your use of this
 * software is governed by the terms of Silicon Labs Master Software License
 * Agreement (MSLA) available at
 * www.silabs.com/about-us/legal/master-software-license-agreement.  This
 * software is distributed to you in Source Code format and is governed by the
 * sections of the MSLA applicable to Source Code.
 *
 ******************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "cmsis_os2.h"
#include "sl_cmsis_os2_common.h"

#include "secure_link/sl_wfx_secure_link.h"

// Securelink Task Configurations
#define SL_WFX_SECURELINK_TASK_PRIO       osPriorityLow
#define SL_WFX_SECURELINK_STACK_SIZE      512u

__ALIGNED(8) static uint8_t sl_wfx_securelink_stack[(SL_WFX_SECURELINK_STACK_SIZE * sizeof(void *)) & 0xFFFFFFF8u];
__ALIGNED(4) static uint8_t sl_wfx_securelink_task_cb[osThreadCbSize];

static uint8_t sl_wfx_securelink_mutex_cb[osMutexCbSize];
static uint8_t sl_wfx_securelink_sem_cb[osSemaphoreCbSize];

osMutexId_t sl_wfx_securelink_rx_mutex;
osSemaphoreId_t sl_wfx_securelink_sem;

/*
 * The task that implements the securelink renegotiation with WFX.
 */
static void sl_wfx_securelink_task(void *p_arg)
{
  sl_status_t result;
  (void)p_arg;

  for (;; ) {
    osSemaphoreAcquire(sl_wfx_securelink_sem, portMAX_Delay);
    result = sl_wfx_secure_link_renegotiate_session_key();
    if (result != SL_STATUS_OK) {
      printf ("WFX session key negotiation error %lu\n",result);
    }
  }
}

/***************************************************************************//**
 * @brief Creates WF200 securelink key renegotiation task.
 ******************************************************************************/
void sl_wfx_securelink_task_start(void)
{
  osThreadId_t       thread_id;
  osThreadAttr_t     thread_attr;
  osMutexAttr_t      mutex_attr
  osSemaphoreAttr_t  sem_attr
  
  mutex_attr.name = "WFX SecureLink RX mutex";
  mutex_attr.cb_mem = sl_wfx_securelink_mutex_cb;
  mutex_attr.cb_size = osMutexCbSize;
  mutex_attr.attr_bits = 0;
  
  sl_wfx_securelink_rx_mutex = osMutexNew(&mutex_attr);
  EFM_ASSERT(sl_wfx_securelink_rx_mutex != NULL);
  
  sem_attr.name = "WFX SecureLink semaphore";
  sem_attr.cb_mem = sl_wfx_securelink_sem_cb;
  sem_attr.cb_size = osSemaphoreCbSize;
  sem_attr.attr_bits = 0;
  
  sl_wfx_securelink_sem = osSemaphoreNew(&sem_attr);
  EFM_ASSERT(sl_wfx_securelink_sem != NULL);
  
  thread_attr.name = "WFX SecureLink task";
  thread_attr.priority = SL_WFX_SECURELINK_TASK_PRIO;
  thread_attr.stack_mem = sl_wfx_securelink_task;
  thread_attr.stack_size = SL_WFX_SECURELINK_STACK_SIZE;
  thread_attr.cb_mem = sl_wfx_securelink_task_cb;
  thread_attr.cb_size = osThreadCbSize;
  thread_attr.attr_bits = 0u;
  thread_attr.tz_module = 0u;
  
  thread_id = osThreadNew(sl_wfx_bus_task, NULL, &thread_attr);
  EFM_ASSERT(thread_id != NULL);
}
