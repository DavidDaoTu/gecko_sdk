#ifdef  SL_WFX_USE_SDIO

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "em_device.h"
#include "em_gpio.h"
#include "em_usart.h"
#include "em_cmu.h"
#include "em_ldma.h"
#include "em_bus.h"
// #include "sleep.h" ??

#include "sdiodrv.h"
#include "sdio.h"

#include "cmsis_os2.h"
#include "sl_cmsis_os2_common.h"
#include "semphr.h"

#include "app_webpage.h"

#include "sl_wfx.h"
#include "sl_wfx_host.h"
#include "sl_wfx_host_pinout.h"
#include "sl_wfx_task.h"

// add
// #include  <rtos_description.h>
#include "sl_wfx.h"
// #include <common/source/kal/kal_priv.h>

// #include <io/include/sd.h>
// #include <io/include/sd_card.h>

// #ifdef SLEEP_ENABLED
// #include "sl_power_manager.h"
// #endif

// #ifdef LIB_MEM_CFG_HEAP_SIZE
// #if (LIB_MEM_CFG_HEAP_SIZE < 20480)
// #error "LIB_MEM_CFG_HEAP_SIZE needs to be >= 20480"
// #endif
// #else
// #error "LIB_MEM_CFG_HEAP_SIZE config missing"
// #endif

// #ifndef SL_WIFI_CFG_SD_CONTROLLER_NAME
// #define SL_WIFI_CFG_SD_CONTROLLER_NAME             "sd0"
// #endif
//

#define SDIO_ACTION_COMPLETION_TIMEOUT_MS     5000

static void com_evt_callback(SDIODRV_Event_t evt, uint32_t error);
static void sdio_irq_callback(void);
static void yield_function(void);

static SDIODRV_Init_t sdiodrv_init = {
  .instance = SDIO,
  .freq = 50000000,
  .portLocationClk = 0,
  .portLocationCmd = 0,
  .portLocationCd = 0,
  .portLocationWp = 0,
  .portLocationDat = 0,
  .clockSource = cmuSelect_AUXHFRCO,
  .transferWidth = SDIO_TRANSFER_WIDTH_4BIT,
  .yield_fn = yield_function
};

static SDIODRV_Handle_t sdiodrv_handle;
static SDIODRV_Callbacks_t sdiodrv_callbacks;

static uint16_t rca;
static bool sdio_enabled = false;
static osSemaphoreId_t sdio_sem_handle;
static uint8_t         sdio_sem_cb[osSemaphoreCbSize];

static SDIODRV_Event_t waited_evt;
static bool sdio_error = false;
static long xHigherPriorityTaskWoken = false;
#ifdef SLEEP_ENABLED
static bool useWIRQ = false;
uint8_t wirq_irq_nb = SL_WFX_HOST_PINOUT_GPIO_WIRQ_PIN;
#endif
bool wfx_interrupt_enabled = false;

/****************************************************************************************************//**
 *                                     sl_wfx_host_init_bus()
 *
 * @brief    Initializes the communications bus.
 *
 * @return   sl_status_t    Error code of the operation as defined in sl_status.h.
 *******************************************************************************************************/
sl_status_t sl_wfx_host_init_bus(void)
{
  sl_status_t status = SL_STATUS_FAIL;
  int res;
  osSemaphoreAttr_t  sdio_sem_attr;

#ifdef SLEEP_ENABLED
  SLEEP_SleepBlockBegin(sleepEM2);
#endif
  printf("\nSDIO init \r\n");
  // Initialize SDIO driver
  res = SDIODRV_Init(&sdiodrv_handle, &sdiodrv_init);
  if (res == 0) {
    res = SDIODRV_DeviceInitAndIdent(&sdiodrv_handle, &rca);
    if (res == 0) {
      res = SDIODRV_SelectCard(&sdiodrv_handle, rca);
      if (res == 0) {
        sdiodrv_callbacks.comEvtCb = com_evt_callback;
        sdiodrv_callbacks.cardInterruptCb = sdio_irq_callback;
        SDIODRV_RegisterCallbacks(&sdiodrv_callbacks);

        // osSemaphoreDef(sdio_sem);
        // sdio_sem_handle = osSemaphoreCreate(osSemaphore(sdio_sem), 1);

        sdio_sem_attr.name = "sdio_sem";
        sdio_sem_attr.cb_mem = sdio_sem_cb;
        sdio_sem_attr.cb_size = osSemaphoreCbSize;
        sdio_sem_attr.attr_bits = 0;
        sdio_sem_handle = osSemaphoreNew(1, 0, &sdio_sem_attr);
        EFM_ASSERT(sdio_sem_handle != NULL);

        if (sdio_sem_handle != NULL) {
          osSemaphoreAcquire(sdio_sem_handle, 0);
          sdio_enabled = true;
          status = SL_STATUS_OK;
        }
      }
    }
  }
  printf("\n sdiodrv_handle = %d\r\n", sdiodrv_handle.init);
  printf("\nSDIO init done res = %d, status = %d, sdio_sem_handle = %d\r\n", res, status, sdio_sem_handle);
  return status;
}

/****************************************************************************************************//**
 *                                     sl_wfx_host_deinit_bus()
 *
 * @brief    De-initializes the communications bus.
 *
 * @return   sl_status_t    Error code of the operation as defined in sl_status.h.
 *******************************************************************************************************/
sl_status_t sl_wfx_host_deinit_bus(void)
{
  sl_status_t status = SL_STATUS_FAIL;
  int res;

  res = SDIODRV_DeInit(&sdiodrv_handle);
  if (res == 0) {
    osSemaphoreDelete(sdio_sem_handle);
    status = SL_STATUS_OK;
  }

  return status;
}

static void com_evt_callback(SDIODRV_Event_t evt, uint32_t error)
{
  // Notify the waiting task
  if ((evt & waited_evt) != 0) {
    osSemaphoreRelease(sdio_sem_handle);
  } else if ((evt & SDIODRV_EVENT_COM_ERROR) != 0) {
    osSemaphoreRelease(sdio_sem_handle);
    sdio_error = true;
  }
}

sl_status_t sl_wfx_host_sdio_transfer_cmd52(sl_wfx_host_bus_transfer_type_t type, uint8_t function, uint32_t address, uint8_t* buffer)
{
  sl_status_t status = SL_STATUS_OK;
    printf("\n sl_wfx_host_sdio_transfer_cmd52 \r\n");
  sdio_error = false;
  waited_evt = SDIODRV_EVENT_CMD_COMPLETE;

  if (type == SL_WFX_BUS_READ) {
    printf("\n sl_wfx_host_sdio_transfer_cmd52 end 1\r\n");
    SDIODRV_IOReadWriteDirect(&sdiodrv_handle, SDIODRV_IO_OP_READ, function, address, buffer);
    printf("\n sl_wfx_host_sdio_transfer_cmd52 end 1\r\n");
  } else {
    SDIODRV_IOReadWriteDirect(&sdiodrv_handle, SDIODRV_IO_OP_WRITE, function, address, buffer);
    printf("\n sl_wfx_host_sdio_transfer_cmd52 end 1\r\n");
  }
  printf("\n sl_wfx_host_sdio_transfer_cmd52 end 1\r\n");
  // Wait for the operation completion
  // portTICK_PERIOD_MS = 1000/1000
  // SDIO_ACTION_COMPLETION_TIMEOUT_MS
  if ((osSemaphoreAcquire(sdio_sem_handle, SDIO_ACTION_COMPLETION_TIMEOUT_MS/portTICK_PERIOD_MS) != osOK)
      || sdio_error) {
    SDIODRV_Abort(&sdiodrv_handle, function);
    status = SL_STATUS_FAIL;
  }
  printf("\n sl_wfx_host_sdio_transfer_cmd52 end\r\n");
  return status;
}

sl_status_t sl_wfx_host_sdio_transfer_cmd53(sl_wfx_host_bus_transfer_type_t type, uint8_t function, uint32_t address, uint8_t* buffer, uint16_t buffer_length)
{
  sl_status_t status = SL_STATUS_OK;
  uint8_t *buf_tmp = buffer;
  uint32_t dummy_data;
  uint16_t block_count;

  sdio_error = false;
  waited_evt = SDIODRV_EVENT_TRANS_COMPLETE;

  // Ensure a valid buffer address for each operations (for SDIO DMA).
  if (buffer == NULL) {
    buf_tmp = (uint8_t *)&dummy_data;
  }

  if (buffer_length >= 512) {
    block_count = (buffer_length / SL_WFX_SDIO_BLOCK_SIZE) + ( ( (buffer_length % SL_WFX_SDIO_BLOCK_SIZE) == 0) ? 0 : 1);

    SDIO_ConfigureTransfer(sdiodrv_handle.init.instance, SL_WFX_SDIO_BLOCK_SIZE, block_count);

    if (type == SL_WFX_BUS_READ) {
      SDIODRV_IOReadWriteExtendedBlocks(&sdiodrv_handle, SDIODRV_IO_OP_READ, function, address, block_count, buf_tmp);
    } else {
      SDIODRV_IOReadWriteExtendedBlocks(&sdiodrv_handle, SDIODRV_IO_OP_WRITE, function, address, block_count, buf_tmp);
    }
  } else {

    SDIO_ConfigureTransfer(sdiodrv_handle.init.instance, buffer_length, 0);

    if (type == SL_WFX_BUS_READ) {
      SDIODRV_IOReadWriteExtendedBytes(&sdiodrv_handle, SDIODRV_IO_OP_READ, function, address, buffer_length, buf_tmp);
    } else {
      SDIODRV_IOReadWriteExtendedBytes(&sdiodrv_handle, SDIODRV_IO_OP_WRITE, function, address, buffer_length, buf_tmp);
    }
  }

  // Wait for the operation completions
  // portTICK_PERIOD_MS = 1000/1000
  if ((osSemaphoreAcquire(sdio_sem_handle, SDIO_ACTION_COMPLETION_TIMEOUT_MS/portTICK_PERIOD_MS) != osOK)
      || sdio_error) {
    SDIODRV_Abort(&sdiodrv_handle, function);
    status = SL_STATUS_FAIL;
  }

  return status;
}

sl_status_t sl_wfx_host_sdio_enable_high_speed_mode(void)
{
  SDIODRV_EnableHighSpeed(&sdiodrv_handle, 1);
  return SL_STATUS_OK;
}

static void sdio_irq_callback(void)
{
  // xSemaphoreGiveFromISR(sl_wfx_wakeup_sem, &xHigherPriorityTaskWoken);
  // xEventGroupSetBitsFromISR(sl_wfx_bus_events,
  //                               2,
  //                               &xHigherPriorityTaskWoken);
  printf("1 sdio_irq_callback\r\n");
  osSemaphoreRelease(sl_wfx_wakeup_sem);
  osEventFlagsSet(sl_wfx_bus_events, SL_WFX_BUS_EVENT_FLAG_RX);
}

static void yield_function(void)
{
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  xHigherPriorityTaskWoken = false;
}

/****************************************************************************************************//**
 *                                  sl_wfx_host_enable_platform_interrupt()
 *
 * @brief    Enable interrupts on the host side.
 *
 * @return   sl_status_t    Error code of the operation as defined in sl_status.h.
 *******************************************************************************************************/
sl_status_t sl_wfx_host_enable_platform_interrupt(void)
{
#ifdef SLEEP_ENABLED
  if (useWIRQ) {
    uint32_t interrupts = GPIO_IntGet();

    GPIO_ExtIntConfig(SL_WFX_HOST_CFG_WIRQPORT,
                      SL_WFX_HOST_CFG_WIRQPIN,
                      SL_WFX_HOST_CFG_IRQ,
                      true,
                      false,
                      true);

    if (((interrupts & 0x400) != 0)
        || (GPIO_PinInGet(SL_WFX_HOST_CFG_WIRQPORT, SL_WFX_HOST_CFG_WIRQPIN) == 1)) {
      GPIO_IntSet(interrupts | 0x400);
    }
  }
  else
#endif
  {
    SDIODRV_EnableInterrupts(&sdiodrv_handle,
                             SDIO_IEN_CARDINTSEN | SDIO_IFENC_CARDINTEN,
                             1);
  }
  printf("\n sl_wfx_host_enable_platform_interrupt \r\n");
  wfx_interrupt_enabled = true;
  return SL_STATUS_OK;
}

/****************************************************************************************************//**
 *                                 sl_wfx_host_disable_platform_interrupt()
 *
 * @brief    Disable interrupts on the host side.
 *
 * @return   sl_status_t    Error code of the operation as defined in sl_status.h.
 *******************************************************************************************************/
sl_status_t sl_wfx_host_disable_platform_interrupt(void)
{
#ifdef SLEEP_ENABLED
  if (useWIRQ) {
    GPIO_IntDisable(SL_WFX_HOST_CFG_IRQ);
  }
  else
#endif
  {
    SDIODRV_EnableInterrupts(&sdiodrv_handle,
                             SDIO_IEN_CARDINTSEN | SDIO_IFENC_CARDINTEN,
                             0);
  }
  wfx_interrupt_enabled = false;
  return SL_STATUS_OK;
}

#ifdef SLEEP_ENABLED
sl_status_t sl_wfx_host_switch_to_wirq(void)
{
  uint32_t value32;

  GPIO_ExtIntConfig(SL_WFX_HOST_CFG_WIRQPORT,
                    SL_WFX_HOST_CFG_WIRQPIN,
                    SL_WFX_HOST_CFG_IRQ,
                    true,
                    false,
                    true);

  sl_wfx_reg_read_32(SL_WFX_CONFIG_REG_ID, &value32);
  value32 |= (1 << 15);
  sl_wfx_reg_write_32(SL_WFX_CONFIG_REG_ID, value32);
  useWIRQ = true;
  return SL_STATUS_OK;
}

sl_status_t sl_wfx_host_enable_sdio(void)
{
  if (sdio_enabled == false) {
    SLEEP_SleepBlockBegin(sleepEM2);
    CMU_OscillatorEnable(cmuOsc_AUXHFRCO, true, true);
    SDIODRV_Enable(&sdiodrv_handle, true);
    sdio_enabled = true;
  }
  return SL_STATUS_OK;
}

sl_status_t sl_wfx_host_disable_sdio(void)
{
  if (sdio_enabled == true) {
    SDIODRV_Enable(&sdiodrv_handle, false);
    CMU_OscillatorEnable(cmuOsc_AUXHFRCO, false, true);
    sdio_enabled = false;
    SLEEP_SleepBlockEnd(sleepEM2);
  }
  return SL_STATUS_OK;
}
#endif

#endif
