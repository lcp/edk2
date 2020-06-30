/** @file

  Macros and type definitions for LSI 53C895A SCSI devices.

  Copyright (C) 2020, SUSE LLC.

  SPDX-License-Identifier: BSD-2-Clause-Patent

**/

#ifndef _LSI_SCSI_H_
#define _LSI_SCSI_H_

//
// Device ID
//
#define LSI_LOGIC_PCI_VENDOR_ID   0x1000
#define LSI_53C895A_PCI_DEVICE_ID 0x0012

//
// LSI 53C895A Registers
//
#define LSI_REG_DSTAT             0x0C
#define LSI_REG_ISTAT0            0x14
#define LSI_REG_DSP               0x2C
#define LSI_REG_SIST0             0x42
#define LSI_REG_SIST1             0x43

//
// The status bits for DMA Status (DSTAT)
//
#define LSI_DSTAT_IID             0x01
#define LSI_DSTAT_R               0x02
#define LSI_DSTAT_SIR             0x04
#define LSI_DSTAT_SSI             0x08
#define LSI_DSTAT_ABRT            0x10
#define LSI_DSTAT_BF              0x20
#define LSI_DSTAT_MDPE            0x40
#define LSI_DSTAT_DFE             0x80

//
// The status bits for Interrupt Status Zero (ISTAT0)
//
#define LSI_ISTAT0_DIP            0x01
#define LSI_ISTAT0_SIP            0x02
#define LSI_ISTAT0_INTF           0x04
#define LSI_ISTAT0_CON            0x08
#define LSI_ISTAT0_SEM            0x10
#define LSI_ISTAT0_SIGP           0x20
#define LSI_ISTAT0_SRST           0x40
#define LSI_ISTAT0_ABRT           0x80

//
// LSI 53C895A Script Instructions
//
#define LSI_INS_TYPE_BLK          0x00000000
#define LSI_INS_TYPE_IO           0x40000000
#define LSI_INS_TYPE_TC           0x80000000

#define LSI_INS_BLK_SCSIP_DAT_OUT 0x00000000
#define LSI_INS_BLK_SCSIP_DAT_IN  0x01000000
#define LSI_INS_BLK_SCSIP_CMD     0x02000000
#define LSI_INS_BLK_SCSIP_STAT    0x03000000
#define LSI_INS_BLK_SCSIP_MSG_OUT 0x06000000
#define LSI_INS_BLK_SCSIP_MSG_IN  0x07000000

#define LSI_INS_IO_OPC_SEL        0x00000000
#define LSI_INS_IO_OPC_WAIT_RESEL 0x10000000

#define LSI_INS_TC_CP             0x00020000
#define LSI_INS_TC_JMP            0x00080000
#define LSI_INS_TC_RA             0x00800000

#define LSI_INS_TC_OPC_JMP        0x00000000
#define LSI_INS_TC_OPC_INT        0x18000000

#define LSI_INS_TC_SCSIP_DAT_OUT  0x00000000
#define LSI_INS_TC_SCSIP_MSG_IN   0x07000000

#endif // _LSI_SCSI_H_
