/***************************************************************************//**
 * @file
 * @brief File System - Ram Disk Media Driver
 *******************************************************************************
 * # License
 * <b>Copyright 2018 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * The licensor of this software is Silicon Laboratories Inc.  Your use of this
 * software is governed by the terms of Silicon Labs Master Software License
 * Agreement (MSLA) available at
 * www.silabs.com/about-us/legal/master-software-license-agreement.
 * The software is governed by the sections of the MSLA applicable to Micrium
 * Software.
 *
 ******************************************************************************/

/********************************************************************************************************
 ********************************************************************************************************
 *                                               MODULE
 ********************************************************************************************************
 *******************************************************************************************************/

#ifndef  FS_RAMDISK_PRIV_H_
#define  FS_RAMDISK_PRIV_H_

/********************************************************************************************************
 ********************************************************************************************************
 *                                               INCLUDE FILES
 ********************************************************************************************************
 *******************************************************************************************************/

#include  <cpu/include/cpu.h>
#include  <common/include/rtos_err.h>
#include  <fs/include/fs_ramdisk.h>

/********************************************************************************************************
 ********************************************************************************************************
 *                                           FUNCTION PROTOTYPES
 ********************************************************************************************************
 *******************************************************************************************************/

FS_RAM_DISK *FS_RAM_Disk_Create(const FS_RAM_DISK_CFG *p_cfg,
                                MEM_SEG               *p_seg,
                                RTOS_ERR              *p_err);

void FS_RAM_Disk_Register(const CPU_CHAR *name,
                          FS_RAM_DISK    *p_ram_disk,
                          RTOS_ERR       *p_err);

void FS_RAM_Disk_Unregister(FS_RAM_DISK *p_ram_disk);

void FS_RAM_Disk_Connect(FS_RAM_DISK *p_ram_disk);

void FS_RAM_Disk_Disconnect(FS_RAM_DISK *p_ram_disk);

const CPU_CHAR *FS_RAM_Disk_NameGet(FS_RAM_DISK *p_ram_disk);

/********************************************************************************************************
 ********************************************************************************************************
 *                                               MODULE END
 ********************************************************************************************************
 *******************************************************************************************************/

#endif