/***************************************************************************//**
 * @file sl_wfx_host_spi.c
 * @brief WFX SPI interface driver implementation
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

#include "sl_wfx_configuration_defaults.h"

#ifdef  SL_WFX_USE_SPI
#include  <rtos_description.h>

#include "sl_wfx.h"
#include "sl_wfx_host_api.h"
#include "sl_wfx_host_pinout.h"

#include "em_usart.h"
#include "em_eusart.h"
#include "em_cmu.h"
#include "dmadrv.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <kernel/include/os.h>
#include <common/include/rtos_utils.h>
#include <common/include/rtos_err.h>
#ifdef SLEEP_ENABLED
#include "sl_power_manager.h"
#endif

#if defined(USART_PRESENT)
#define USART           SL_WFX_HOST_PINOUT_SPI_PERIPHERAL
#elif defined(EUSART_PRESENT)
#define EUSART          SL_WFX_HOST_PINOUT_SPI_PERIPHERAL
#endif

static OS_SEM spi_sem;
static unsigned int        tx_dma_channel;
static unsigned int        rx_dma_channel;
static uint32_t            dummy_rx_data;
static uint32_t            dummy_tx_data;
#if defined(USART_PRESENT)
static uint32_t            usart_clock;
static uint32_t            usart_rx_signal;
static uint32_t            usart_tx_signal;
#endif
#if defined(EUSART_PRESENT)
static uint32_t            eusart_clock;
static uint32_t            eusart_rx_signal;
static uint32_t            eusart_tx_signal;
#endif
static bool spi_enabled = false;

uint8_t wirq_irq_nb = SL_WFX_HOST_PINOUT_SPI_WIRQ_PIN;

static int sl_wfx_host_spi_set_config(void *usart)
{
  int ret = -1;

  if ( 0 ) {
#if defined(USART0)
  } else if (usart == USART0) {
    usart_clock     = cmuClock_USART0;
    usart_tx_signal = dmadrvPeripheralSignal_USART0_TXBL;
    usart_rx_signal = dmadrvPeripheralSignal_USART0_RXDATAV;
    ret = 0;
#endif
#if defined(USART1)
  } else if (usart == USART1 ) {
    usart_clock     = cmuClock_USART1;
    usart_tx_signal = dmadrvPeripheralSignal_USART1_TXBL;
    usart_rx_signal = dmadrvPeripheralSignal_USART1_RXDATAV;
    ret = 0;
#endif
#if defined(USART2)
  } else if (usart == USART2 ) {
    usart_clock     = cmuClock_USART2;
    usart_tx_signal = dmadrvPeripheralSignal_USART2_TXBL;
    usart_rx_signal = dmadrvPeripheralSignal_USART2_RXDATAV;
    ret = 0;
#endif
#if defined(USART3)
  } else if (usart == USART3 ) {
    usart_clock     = cmuClock_USART3;
    usart_tx_signal = dmadrvPeripheralSignal_USART3_TXBL;
    usart_rx_signal = dmadrvPeripheralSignal_USART3_RXDATAV;
    ret = 0;
#endif
#if defined(USART4)
  } else if (usart == USART4 ) {
    usart_clock     = cmuClock_USART4;
    usart_tx_signal = dmadrvPeripheralSignal_USART4_TXBL;
    usart_rx_signal = dmadrvPeripheralSignal_USART4_RXDATAV;
    ret = 0;
#endif
#if defined(USART5)
  } else if (usart == USART5 ) {
    usart_clock     = cmuClock_USART5;
    usart_tx_signal = dmadrvPeripheralSignal_USART5_TXBL;
    usart_rx_signal = dmadrvPeripheralSignal_USART5_RXDATAV;
    ret = 0;
#endif
#if defined(USARTRF0)
  } else if (usart == USARTRF0 ) {
    usart_clock     = cmuClock_USARTRF0;
    usart_tx_signal = dmadrvPeripheralSignal_USARTRF0_TXBL;
    usart_rx_signal = dmadrvPeripheralSignal_USARTRF0_RXDATAV;
    ret = 0;
#endif
#if defined(USARTRF1)
  } else if (usart == USARTRF1 ) {
    usart_clock     = cmuClock_USARTRF1;
    usart_tx_signal = dmadrvPeripheralSignal_USARTRF1_TXBL;
    usart_rx_signal = dmadrvPeripheralSignal_USARTRF1_RXDATAV;
    ret = 0;
#endif
#if defined(EUSART1)
  } else if (usart == EUSART1 ) {
    eusart_clock     = cmuClock_EUSART1;
    eusart_tx_signal = dmadrvPeripheralSignal_EUSART1_TXBL;
    eusart_rx_signal = dmadrvPeripheralSignal_EUSART1_RXDATAV;
    ret = 0;
#endif
#if defined(EUSART2)
  } else if (usart == EUSART2 ) {
    eusart_clock     = cmuClock_EUSART2;
    eusart_tx_signal = dmadrvPeripheralSignal_EUSART2_TXBL;
    eusart_rx_signal = dmadrvPeripheralSignal_EUSART2_RXDATAV;
    ret = 0;
#endif
  }

  return ret;
}

/**************************************************************************//**
 * Initialize SPI peripheral
 *****************************************************************************/
sl_status_t sl_wfx_host_init_bus(void)
{
  RTOS_ERR err;
  int res;

#if defined(USART_PRESENT)
  // Initialize and enable the USART
  USART_InitSync_TypeDef usartInit = USART_INITSYNC_DEFAULT;

  res = sl_wfx_host_spi_set_config(USART);
  if (res != 0) {
    return SL_STATUS_FAIL;
  }

  spi_enabled = true;
  dummy_tx_data = 0;
  usartInit.baudrate = 36000000u;
  usartInit.msbf = true;
#if defined(EFR32MG24B020F1536IM48) || defined(EFR32MG24A020F1536GM48) || defined(EFR32MG24B220F1536IM48)
  CMU_ClockEnable(cmuClock_USART0, true);
#elif defined(EFR32MG21A020F1024IM32) || defined(EFR32MG21A010F1024IM32)
  CMU_ClockEnable(cmuClock_USART2, true);
#else
  CMU_ClockEnable(cmuClock_HFPER, true);
#endif
  
  CMU_ClockEnable(cmuClock_GPIO, true);
  CMU_ClockEnable(usart_clock, true);
  USART_InitSync(USART, &usartInit);
  USART->CTRL |= (1u << _USART_CTRL_SMSDELAY_SHIFT);

#if defined(EFR32MG24B020F1536IM48) || defined(EFR32MG24A020F1536GM48) || defined(EFR32MG24B220F1536IM48) \
    || defined(EFR32MG21A020F1024IM32) || defined(EFR32MG21A010F1024IM32)

  GPIO->USARTROUTE[SL_WFX_HOST_PINOUT_SPI_PERIPHERAL_NO].TXROUTE = 
                                (SL_WFX_HOST_PINOUT_SPI_TX_PORT 
                                  << _GPIO_USART_TXROUTE_PORT_SHIFT)
                                | (SL_WFX_HOST_PINOUT_SPI_TX_PIN 
                                  << _GPIO_USART_TXROUTE_PIN_SHIFT);

  GPIO->USARTROUTE[SL_WFX_HOST_PINOUT_SPI_PERIPHERAL_NO].RXROUTE = 
                                (SL_WFX_HOST_PINOUT_SPI_RX_PORT 
                                  << _GPIO_USART_RXROUTE_PORT_SHIFT)
                                | (SL_WFX_HOST_PINOUT_SPI_RX_PIN 
                                  << _GPIO_USART_RXROUTE_PIN_SHIFT);

  GPIO->USARTROUTE[SL_WFX_HOST_PINOUT_SPI_PERIPHERAL_NO].CLKROUTE = 
                                (SL_WFX_HOST_PINOUT_SPI_CLK_PORT 
                                  << _GPIO_USART_CLKROUTE_PORT_SHIFT)
                                | (SL_WFX_HOST_PINOUT_SPI_CLK_PIN 
                                  << _GPIO_USART_CLKROUTE_PIN_SHIFT);

  GPIO->USARTROUTE[SL_WFX_HOST_PINOUT_SPI_PERIPHERAL_NO].ROUTEEN = 
                                GPIO_USART_ROUTEEN_RXPEN  |
                                GPIO_USART_ROUTEEN_TXPEN  |
                                GPIO_USART_ROUTEEN_CLKPEN;

#else
  USART->ROUTELOC0 = (USART->ROUTELOC0
                      & ~(_USART_ROUTELOC0_TXLOC_MASK
                          | _USART_ROUTELOC0_RXLOC_MASK
                          | _USART_ROUTELOC0_CLKLOC_MASK))
                    | (SL_WFX_HOST_PINOUT_SPI_TX_LOC  << _USART_ROUTELOC0_TXLOC_SHIFT)
                    | (SL_WFX_HOST_PINOUT_SPI_RX_LOC  << _USART_ROUTELOC0_RXLOC_SHIFT)
                    | (SL_WFX_HOST_PINOUT_SPI_CLK_LOC << _USART_ROUTELOC0_CLKLOC_SHIFT);

  USART->ROUTEPEN = USART_ROUTEPEN_TXPEN
                    | USART_ROUTEPEN_RXPEN
                    | USART_ROUTEPEN_CLKPEN;
  GPIO_DriveStrengthSet(SL_WFX_HOST_PINOUT_SPI_CLK_PORT, gpioDriveStrengthStrongAlternateStrong);
#endif

#elif defined(EUSART_PRESENT)
  // Initialize and enable the EUSART
  EUSART_SpiInit_TypeDef init = EUSART_SPI_MASTER_INIT_DEFAULT_HF;
  EUSART_SpiAdvancedInit_TypeDef advancedInit = EUSART_SPI_ADVANCED_INIT_DEFAULT;

  res = sl_wfx_host_spi_set_config(EUSART);
  if (res != 0) {
    return SL_STATUS_FAIL;
  }

  spi_enabled = true;
  dummy_tx_data = 0;

  // Configure the clocks.
  // CMU_ClockSelectSet(cmuClock_EUSART2, cmuSelect_EM01GRPCCLK);
  CMU_ClockEnable(cmuClock_GPIO, true);
  CMU_ClockEnable(eusart_clock, true);

  advancedInit.msbFirst = true;
  // advancedInit.autoCsEnable = false;
  // init.bitRate = 7500000;
  init.advancedSettings = &advancedInit;

  // Initialize the EUSART
  EUSART_SpiInit(EUSART, &init);
  // EUSART->CTRL |= (1u << _USART_CTRL_SMSDELAY_SHIFT);

  // Connect EUSART to ports
  GPIO->EUSARTROUTE[SL_WFX_HOST_PINOUT_SPI_PERIPHERAL_NO].TXROUTE = 
                                (SL_WFX_HOST_PINOUT_SPI_TX_PORT 
                                  << _GPIO_EUSART_TXROUTE_PORT_SHIFT)
                                | (SL_WFX_HOST_PINOUT_SPI_TX_PIN 
                                  << _GPIO_EUSART_TXROUTE_PIN_SHIFT);

  GPIO->EUSARTROUTE[SL_WFX_HOST_PINOUT_SPI_PERIPHERAL_NO].RXROUTE = 
                                (SL_WFX_HOST_PINOUT_SPI_RX_PORT 
                                  << _GPIO_EUSART_RXROUTE_PORT_SHIFT)
                                | (SL_WFX_HOST_PINOUT_SPI_RX_PIN 
                                  << _GPIO_EUSART_RXROUTE_PIN_SHIFT);

  GPIO->EUSARTROUTE[SL_WFX_HOST_PINOUT_SPI_PERIPHERAL_NO].SCLKROUTE = 
                                (SL_WFX_HOST_PINOUT_SPI_CLK_PORT 
                                  << _GPIO_EUSART_SCLKROUTE_PORT_SHIFT)
                                | (SL_WFX_HOST_PINOUT_SPI_CLK_PIN 
                                  << _GPIO_EUSART_SCLKROUTE_PIN_SHIFT);

  GPIO->EUSARTROUTE[SL_WFX_HOST_PINOUT_SPI_PERIPHERAL_NO].ROUTEEN = 
                                GPIO_EUSART_ROUTEEN_RXPEN  |
                                GPIO_EUSART_ROUTEEN_TXPEN  |
                                GPIO_EUSART_ROUTEEN_SCLKPEN;

#endif /* USART_PRESENT */

  GPIO_PinModeSet(SL_WFX_HOST_PINOUT_SPI_TX_PORT, SL_WFX_HOST_PINOUT_SPI_TX_PIN, gpioModePushPull, 0);
  GPIO_PinModeSet(SL_WFX_HOST_PINOUT_SPI_RX_PORT, SL_WFX_HOST_PINOUT_SPI_RX_PIN, gpioModeInput, 0);
  GPIO_PinModeSet(SL_WFX_HOST_PINOUT_SPI_CLK_PORT, SL_WFX_HOST_PINOUT_SPI_CLK_PIN, gpioModePushPull, 0);
  OSSemCreate(&spi_sem, "spi semaphore", 0, &err);
  DMADRV_Init();
  DMADRV_AllocateChannel(&tx_dma_channel, NULL);
  DMADRV_AllocateChannel(&rx_dma_channel, NULL);
  GPIO_PinModeSet(SL_WFX_HOST_PINOUT_SPI_CS_PORT, SL_WFX_HOST_PINOUT_SPI_CS_PIN, gpioModePushPull, 1);
#if defined(USART_PRESENT)
  USART->CMD = USART_CMD_CLEARRX | USART_CMD_CLEARTX;
#elif defined(EUSART_PRESENT)
  EUSART->CMD = EUSART_CMD_CLEARTX;
#endif
  return SL_STATUS_OK;
}

/**************************************************************************//**
 * De-initialize SPI peripheral and DMAs
 *****************************************************************************/
sl_status_t sl_wfx_host_deinit_bus(void)
{
  RTOS_ERR err;

  OSSemDel(&spi_sem, OS_OPT_DEL_ALWAYS, &err);
  // Stop DMAs.
  DMADRV_StopTransfer(rx_dma_channel);
  DMADRV_StopTransfer(tx_dma_channel);
  DMADRV_FreeChannel(tx_dma_channel);
  DMADRV_FreeChannel(rx_dma_channel);
  DMADRV_DeInit();
  #if defined(USART_PRESENT)
  USART_Reset(USART);
  #elif defined(EUSART_PRESENT)
  EUSART_Reset(EUSART);
  #endif
  return SL_STATUS_OK;
}

/**************************************************************************//**
 * Assert chip select.
 *****************************************************************************/
sl_status_t sl_wfx_host_spi_cs_assert()
{
  GPIO_PinOutClear(SL_WFX_HOST_PINOUT_SPI_CS_PORT, SL_WFX_HOST_PINOUT_SPI_CS_PIN);
  return SL_STATUS_OK;
}

/**************************************************************************//**
 * De-assert chip select.
 *****************************************************************************/
sl_status_t sl_wfx_host_spi_cs_deassert()
{
  GPIO_PinOutSet(SL_WFX_HOST_PINOUT_SPI_CS_PORT, SL_WFX_HOST_PINOUT_SPI_CS_PIN);
  return SL_STATUS_OK;
}

static bool rx_dma_complete(unsigned int channel,
                            unsigned int sequenceNo,
                            void *userParam)
{
  (void)channel;
  (void)sequenceNo;
  (void)userParam;
  RTOS_ERR err;
  OSSemPost(&spi_sem, OS_OPT_POST_1, &err);
  return true;
}

void receiveDMA(uint8_t* buffer, uint16_t buffer_length)
{
#if defined(USART_PRESENT)
  // Start receive DMA.
  DMADRV_PeripheralMemory(rx_dma_channel,
                          usart_rx_signal,
                          (void*)buffer,
                          (void *)&(USART->RXDATA),
                          true,
                          buffer_length,
                          dmadrvDataSize1,
                          rx_dma_complete,
                          NULL);

  // Start transmit DMA.
  DMADRV_MemoryPeripheral(tx_dma_channel,
                          usart_tx_signal,
                          (void *)&(USART->TXDATA),
                          (void *)&(dummy_tx_data),
                          false,
                          buffer_length,
                          dmadrvDataSize1,
                          NULL,
                          NULL);
#elif defined(EUSART_PRESENT) || defined(EFR32FG25B222F1920IM56)
  // Start receive DMA.
  DMADRV_PeripheralMemory(rx_dma_channel,
                          eusart_rx_signal,
                          (void*)buffer,
                          (void *)&(EUSART->RXDATA),
                          true,
                          buffer_length,
                          dmadrvDataSize1,
                          rx_dma_complete,
                          NULL);

  // Start transmit DMA.
  DMADRV_MemoryPeripheral(tx_dma_channel,
                          eusart_tx_signal,
                          (void *)&(EUSART->TXDATA),
                          (void *)&(dummy_tx_data),
                          false,
                          buffer_length,
                          dmadrvDataSize1,
                          NULL,
                          NULL);
#endif /* USART_PRESENT */
}

void transmitDMA(uint8_t* buffer, uint16_t buffer_length)
{
#if defined(USART_PRESENT)
  // Receive DMA runs only to initiate callback
  // Start receive DMA.
  DMADRV_PeripheralMemory(rx_dma_channel,
                          usart_rx_signal,
                          &dummy_rx_data,
                          (void *)&(USART->RXDATA),
                          false,
                          buffer_length,
                          dmadrvDataSize1,
                          rx_dma_complete,
                          NULL);
  // Start transmit DMA.
  DMADRV_MemoryPeripheral(tx_dma_channel,
                          usart_tx_signal,
                          (void *)&(USART->TXDATA),
                          (void*)buffer,
                          true,
                          buffer_length,
                          dmadrvDataSize1,
                          NULL,
                          NULL);
#elif defined(EUSART_PRESENT) || defined(EFR32FG25B222F1920IM56)
  DMADRV_PeripheralMemory(rx_dma_channel,
                          eusart_rx_signal,
                          &dummy_rx_data,
                          (void *)&(EUSART->RXDATA),
                          false,
                          buffer_length,
                          dmadrvDataSize1,
                          rx_dma_complete,
                          NULL);
  // Start transmit DMA.
  DMADRV_MemoryPeripheral(tx_dma_channel,
                          eusart_tx_signal,
                          (void *)&(EUSART->TXDATA),
                          (void*)buffer,
                          true,
                          buffer_length,
                          dmadrvDataSize1,
                          NULL,
                          NULL);
#endif /* USART_PRESENT */

}

/**************************************************************************//**
 * WFX SPI transfer implementation
 *****************************************************************************/
sl_status_t sl_wfx_host_spi_transfer_no_cs_assert(sl_wfx_host_bus_transfer_type_t type,
                                                  uint8_t *header,
                                                  uint16_t header_length,
                                                  uint8_t *buffer,
                                                  uint16_t buffer_length)
{
  const bool is_read = (type == SL_WFX_BUS_READ);
  RTOS_ERR err;
  err.Code = RTOS_ERR_NONE;

#if defined(USART_PRESENT)
  // Check that transmit buffer is empty
  while (!(USART->STATUS & USART_STATUS_TXBL)) {
  }
  USART->CMD = USART_CMD_CLEARRX | USART_CMD_CLEARTX;

  if (header_length > 0) {
    for (uint8_t *buffer_ptr = header; header_length > 0; --header_length, ++buffer_ptr) {
      USART->TXDATA = (uint32_t)(*buffer_ptr);

      while (!(USART->STATUS & USART_STATUS_TXC)) {
      }
    }
    // Check that transmit buffer is empty
    while (!(USART->STATUS & USART_STATUS_TXBL)) {
    }
  }
  if (buffer_length > 0) {
    USART->CMD = USART_CMD_CLEARRX | USART_CMD_CLEARTX;
    OSSemSet(&spi_sem, 0, &err);
    if (is_read) {
      receiveDMA(buffer, buffer_length);
    } else {
      transmitDMA(buffer, buffer_length);
    }
    OSSemPend(&spi_sem, 0, OS_OPT_PEND_BLOCKING, 0, &err);
  }
#elif defined(EUSART_PRESENT) || defined(EFR32FG25B222F1920IM56)
  // Check that transmit FIFO is not full
  while (!(EUSART->STATUS & EUSART_STATUS_TXFL)) {
  }
  EUSART->CMD = EUSART_CMD_CLEARTX;

  if (header_length > 0) {
    for (uint8_t *buffer_ptr = header; header_length > 0; --header_length, ++buffer_ptr) {
      EUSART->TXDATA = (uint32_t)(*buffer_ptr);

      while (!(EUSART->STATUS & EUSART_STATUS_TXC)) {
      }
    }
    // Check that transmit FIFO is not full
    while (!(EUSART->STATUS & EUSART_STATUS_TXFL)) {
    }
  }
  if (buffer_length > 0) {
    EUSART->CMD = EUSART_CMD_CLEARTX;
    OSSemSet(&spi_sem, 0, &err);
    if (is_read) {
      receiveDMA(buffer, buffer_length);
    } else {
      transmitDMA(buffer, buffer_length);
    }
    OSSemPend(&spi_sem, 0, OS_OPT_PEND_BLOCKING, 0, &err);
  }
#endif /* USART_PRESENT */

  if (err.Code == RTOS_ERR_NONE) {
    return SL_STATUS_OK;
  } else {
    return SL_STATUS_FAIL;
  }
}

/**************************************************************************//**
 * Enable WFX interrupt
 *****************************************************************************/
sl_status_t sl_wfx_host_enable_platform_interrupt(void)
{
  GPIO_ExtIntConfig(SL_WFX_HOST_PINOUT_SPI_WIRQ_PORT,
                    SL_WFX_HOST_PINOUT_SPI_WIRQ_PIN,
                    wirq_irq_nb,
                    true,
                    false,
                    true);

  return SL_STATUS_OK;
}

/**************************************************************************//**
 * Disable WFX interrupt
 *****************************************************************************/
sl_status_t sl_wfx_host_disable_platform_interrupt(void)
{
  GPIO_IntDisable(1 << wirq_irq_nb);
  return SL_STATUS_OK;
}

sl_status_t sl_wfx_host_enable_spi(void)
{
  if (spi_enabled == false) {
#ifdef SLEEP_ENABLED
    // Prevent the host to use lower EM than EM1
    sl_power_manager_add_em_requirement(SL_POWER_MANAGER_EM1);
#endif
    spi_enabled = true;
  }
  return SL_STATUS_OK;
}

sl_status_t sl_wfx_host_disable_spi(void)
{
  if (spi_enabled == true) {
    spi_enabled = false;
#ifdef SLEEP_ENABLED
    // Allow the host to use the lowest allowed EM
    sl_power_manager_remove_em_requirement(SL_POWER_MANAGER_EM1);
#endif
  }
  return SL_STATUS_OK;
}

#endif
