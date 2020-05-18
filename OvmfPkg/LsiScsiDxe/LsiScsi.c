/** @file

  This driver produces Extended SCSI Pass Thru Protocol instances for
  LSI 53C895A SCSI devices.

  Copyright (C) 2020, SUSE LLC.

  SPDX-License-Identifier: BSD-2-Clause-Patent

**/

#include <IndustryStandard/LsiScsi.h>
#include <IndustryStandard/Pci.h>
#include <Library/BaseLib.h>
#include <Library/BaseMemoryLib.h>
#include <Library/DebugLib.h>
#include <Library/MemoryAllocationLib.h>
#include <Library/PcdLib.h>
#include <Library/UefiBootServicesTableLib.h>
#include <Library/UefiLib.h>
#include <Protocol/PciIo.h>
#include <Protocol/PciRootBridgeIo.h>
#include <Protocol/ScsiPassThruExt.h>
#include <Uefi/UefiSpec.h>

//
// Runtime Structures
//
typedef struct {
  //
  // Allocate 32 UINT32 entries for the script and it's sufficient for
  // 16 instructions.
  //
  UINT32                          Script[32];
  //
  // The max size of CDB is 32.
  //
  UINT8                           Cdb[32];
  //
  // Allocate 64KB for read/write buffer
  //
  UINT8                           Data[0x10000];
  UINT8                           MsgIn[2];
  UINT8                           MsgOut;
  UINT8                           Status;
} LSI_SCSI_DMA_BUFFER;

#define LSI_SCSI_DEV_SIGNATURE SIGNATURE_32 ('L','S','I','S')
typedef struct {
  UINT32                          Signature;
  EFI_EXT_SCSI_PASS_THRU_PROTOCOL PassThru;
  EFI_EXT_SCSI_PASS_THRU_MODE     PassThruMode;
  EFI_PCI_IO_PROTOCOL             *PciIo;
  UINT8                           MaxTarget;
  UINT8                           MaxLun;
  UINT32                          StallPerPollUsec;
  UINT64                          OriginalPciAttributes;
  EFI_EVENT                       ExitBoot;
  LSI_SCSI_DMA_BUFFER             *Dma;
  EFI_PHYSICAL_ADDRESS            DmaPhysical;
  VOID                            *DmaMapping;
} LSI_SCSI_DEV;

#define LSI_SCSI_FROM_PASS_THRU(PassThruPtr) \
  CR (PassThruPtr, LSI_SCSI_DEV, PassThru, LSI_SCSI_DEV_SIGNATURE)

#define LSI_SCSI_DMA_ADDR(Dev, MemberName) \
  (Dev->DmaPhysical + OFFSET_OF (LSI_SCSI_DMA_BUFFER, MemberName))

#define LSI_SCSI_DMA_ADDR_LOW(Dev, MemberName) \
  ((UINT32)LSI_SCSI_DMA_ADDR (Dev, MemberName))

STATIC
EFI_STATUS
Out8 (
  IN LSI_SCSI_DEV *Dev,
  IN UINT32       Addr,
  IN UINT8        Data
  )
{
  return Dev->PciIo->Io.Write (
                          Dev->PciIo,
                          EfiPciIoWidthUint8,
                          PCI_BAR_IDX0,
                          Addr,
                          1,
                          &Data
                          );
}

STATIC
EFI_STATUS
Out32 (
  IN LSI_SCSI_DEV       *Dev,
  IN UINT32             Addr,
  IN UINT32             Data
  )
{
  return Dev->PciIo->Io.Write (
                          Dev->PciIo,
                          EfiPciIoWidthUint32,
                          PCI_BAR_IDX0,
                          Addr,
                          1,
                          &Data
                          );
}

STATIC
EFI_STATUS
In8 (
  IN  LSI_SCSI_DEV *Dev,
  IN  UINT32       Addr,
  OUT UINT8        *Data
  )
{
  return Dev->PciIo->Io.Read (
                          Dev->PciIo,
                          EfiPciIoWidthUint8,
                          PCI_BAR_IDX0,
                          Addr,
                          1,
                          Data
                          );
}

STATIC
EFI_STATUS
LsiScsiReset (
  IN LSI_SCSI_DEV *Dev
  )
{
  return Out8 (Dev, LSI_REG_ISTAT0, LSI_ISTAT0_SRST);
}

STATIC
EFI_STATUS
ReportHostAdapterOverrunError (
  OUT EFI_EXT_SCSI_PASS_THRU_SCSI_REQUEST_PACKET *Packet
  )
{
  Packet->SenseDataLength = 0;
  Packet->HostAdapterStatus =
            EFI_EXT_SCSI_STATUS_HOST_ADAPTER_DATA_OVERRUN_UNDERRUN;
  Packet->TargetStatus = EFI_EXT_SCSI_STATUS_TARGET_GOOD;
  return EFI_BAD_BUFFER_SIZE;
}

STATIC
EFI_STATUS
LsiScsiCheckRequest (
  IN LSI_SCSI_DEV                                   *Dev,
  IN UINT8                                          Target,
  IN UINT64                                         Lun,
  IN OUT EFI_EXT_SCSI_PASS_THRU_SCSI_REQUEST_PACKET *Packet
  )
{
  if (Target > Dev->MaxTarget || Lun > Dev->MaxLun ||
      Packet->DataDirection > EFI_EXT_SCSI_DATA_DIRECTION_BIDIRECTIONAL ||
      //
      // Trying to receive, but destination pointer is NULL, or contradicting
      // transfer direction
      //
      (Packet->InTransferLength > 0 &&
       (Packet->InDataBuffer == NULL ||
        Packet->DataDirection == EFI_EXT_SCSI_DATA_DIRECTION_WRITE
         )
        ) ||

      //
      // Trying to send, but source pointer is NULL, or contradicting transfer
      // direction
      //
      (Packet->OutTransferLength > 0 &&
       (Packet->OutDataBuffer == NULL ||
        Packet->DataDirection == EFI_EXT_SCSI_DATA_DIRECTION_READ
         )
        )
    ) {
    return EFI_INVALID_PARAMETER;
  }

  if (Packet->DataDirection == EFI_EXT_SCSI_DATA_DIRECTION_BIDIRECTIONAL ||
      (Packet->InTransferLength > 0 && Packet->OutTransferLength > 0) ||
      Packet->CdbLength > sizeof Dev->Dma->Cdb) {
    return EFI_UNSUPPORTED;
  }

  if (Packet->InTransferLength > sizeof Dev->Dma->Data) {
    Packet->InTransferLength = sizeof Dev->Dma->Data;
    return ReportHostAdapterOverrunError (Packet);
  }
  if (Packet->OutTransferLength > sizeof Dev->Dma->Data) {
    Packet->OutTransferLength = sizeof Dev->Dma->Data;
    return ReportHostAdapterOverrunError (Packet);
  }

  return EFI_SUCCESS;
}

STATIC
EFI_STATUS
LsiScsiProcessRequest (
  IN LSI_SCSI_DEV                                   *Dev,
  IN UINT8                                          Target,
  IN UINT64                                         Lun,
  IN OUT EFI_EXT_SCSI_PASS_THRU_SCSI_REQUEST_PACKET *Packet
  )
{
  EFI_STATUS Status;
  UINT32     *Script;
  UINT8      *Cdb;
  UINT8      *MsgOut;
  UINT8      *MsgIn;
  UINT8      *ScsiStatus;
  UINT8      *Data;
  UINT8      DStat;
  UINT8      SIst0;
  UINT8      SIst1;

  Script      = Dev->Dma->Script;
  Cdb         = Dev->Dma->Cdb;
  Data        = Dev->Dma->Data;
  MsgIn       = Dev->Dma->MsgIn;
  MsgOut      = &Dev->Dma->MsgOut;
  ScsiStatus  = &Dev->Dma->Status;

  *MsgOut     = 0x80 | (UINT8) Lun;
  *ScsiStatus = 0xFF;

  SetMem (Cdb, sizeof Dev->Dma->Cdb, 0x00);
  CopyMem (Cdb, Packet->Cdb, Packet->CdbLength);

  //
  // Set SenseDataLength to 0 for "TEST UNIT READY" to request
  // sense data by "REQUEST SENSE".
  //
  if (Cdb[0] == 0x00) {
    Packet->SenseDataLength = 0;
  }

  //
  // Compose the script for the controller
  // Reference:
  //   LSI53C895A PCI to Ultra2 SCSI Controller Version 2.2
  //   Chapter 5 SCSI SCRIPT Instruction Set
  //
  SetMem (Script, sizeof Dev->Dma->Script, 0x00);

  //
  // Select target
  //
  *Script++ = LSI_INS_TYPE_IO | LSI_INS_IO_OPC_SEL | (UINT32)Target << 16;
  *Script++ = 0x00000000;

  //
  // Select Lun in MsgOut (1 bytes)
  //
  *Script++ = LSI_INS_TYPE_BLK | LSI_INS_BLK_SCSIP_MSG_OUT | 0x1;
  *Script++ = LSI_SCSI_DMA_ADDR_LOW (Dev, MsgOut);

  //
  // Send the SCSI Command
  //
  *Script++ = LSI_INS_TYPE_BLK | LSI_INS_BLK_SCSIP_CMD | 0x10;
  *Script++ = LSI_SCSI_DMA_ADDR_LOW (Dev, Cdb);

  //
  // Handle disconnect
  //
  // Check whether the current SCSI phase is "Message In" and jump to the
  // next section if it is.
  //
  *Script++ = LSI_INS_TYPE_TC | LSI_INS_TC_OPC_JMP | \
              LSI_INS_TC_SCSIP_MSG_IN | LSI_INS_TC_RA | \
              LSI_INS_TC_CP;
  *Script++ = 0x00000018;
  //
  // Set the SCSI phase to "Message In" to trigger reselect
  //
  *Script++ = LSI_INS_TYPE_BLK | LSI_INS_BLK_SCSIP_MSG_IN | 0x2;
  *Script++ = LSI_SCSI_DMA_ADDR_LOW (Dev, MsgIn);
  //
  // Wait reselect
  //
  *Script++ = LSI_INS_TYPE_IO | LSI_INS_IO_OPC_WAIT_RESEL;
  *Script++ = 0x00000000;
  //
  // Set the SCSI phase to "Message In"
  //
  *Script++ = LSI_INS_TYPE_BLK | LSI_INS_BLK_SCSIP_MSG_IN | 0x2;
  *Script++ = LSI_SCSI_DMA_ADDR_LOW (Dev, MsgIn);

  //
  // DMA command for the read/write operations
  //
  if (Packet->DataDirection == EFI_EXT_SCSI_DATA_DIRECTION_READ &&
      Packet->InTransferLength > 0) {
    *Script++ = LSI_INS_TYPE_BLK | LSI_INS_BLK_SCSIP_DAT_IN | \
                Packet->InTransferLength;
    *Script++ = LSI_SCSI_DMA_ADDR_LOW (Dev, Data);
  } else if (Packet->DataDirection == EFI_EXT_SCSI_DATA_DIRECTION_WRITE &&
             Packet->OutTransferLength > 0) {
    // LsiScsiCheckRequest() guarantees that OutTransferLength is no
    // larger than sizeof Dev->Dma->Data, so we can safely copy the
    // the data to Dev->Dma->Data.
    CopyMem (Data, Packet->OutDataBuffer, Packet->OutTransferLength);
    *Script++ = LSI_INS_TYPE_BLK | LSI_INS_BLK_SCSIP_DAT_OUT | \
                Packet->OutTransferLength;
    *Script++ = LSI_SCSI_DMA_ADDR_LOW (Dev, Data);
  }

  //
  // Get the SCSI status
  //
  *Script++ = LSI_INS_TYPE_BLK | LSI_INS_BLK_SCSIP_STAT | 0x1;
  *Script++ = LSI_SCSI_DMA_ADDR_LOW (Dev, Status);

  //
  // Get the SCSI message
  //
  *Script++ = LSI_INS_TYPE_BLK | LSI_INS_BLK_SCSIP_MSG_IN | 0x1,
  *Script++ = LSI_SCSI_DMA_ADDR_LOW (Dev, MsgIn);

  //
  // Raise the interrupt to end the script
  //
  *Script++ = LSI_INS_TYPE_TC | LSI_INS_TC_OPC_INT | \
              LSI_INS_TC_SCSIP_DAT_OUT | LSI_INS_TC_JMP;
  *Script++ = 0x00000000;

  ASSERT (Script < Dev->Dma->Script + sizeof Dev->Dma->Script);

  //
  // Execute the script
  //
  Status = Out32 (Dev, LSI_REG_DSP0, LSI_SCSI_DMA_ADDR_LOW(Dev, Script));
  if (EFI_ERROR (Status)) {
    return Status;
  }

  //
  // Poll the device registers: DSTAT, SIST0, and SIST1
  //
  for(;;) {
    Status = In8 (Dev, LSI_REG_DSTAT, &DStat);
    if (EFI_ERROR (Status)) {
      goto Error;
    }
    Status = In8 (Dev, LSI_REG_SIST0, &SIst0);
    if (EFI_ERROR (Status)) {
      goto Error;
    }
    Status = In8 (Dev, LSI_REG_SIST1, &SIst1);
    if (EFI_ERROR (Status)) {
      goto Error;
    }

    if (SIst0 != 0 || SIst1 != 0) {
      goto Error;
    }

    //
    // Check the SIR bit (SCRIPTS Interrupt Instruction Received)
    // If it is set, the script is finished.
    //
    if (DStat & 0x04) {
      break;
    }

    gBS->Stall (Dev->StallPerPollUsec);
  }

  //
  // Copy Data to InDataBuffer if necessary
  //
  if (Packet->DataDirection == EFI_EXT_SCSI_DATA_DIRECTION_READ) {
    CopyMem (Packet->InDataBuffer, Data, Packet->InTransferLength);
  }

  //
  // Check if everything is good
  //   SCSI Message Code 0x00: COMMAND COMPLETE
  //   SCSI Status Code  0x00: Good
  //
  if (MsgIn[0] == 0 && *ScsiStatus == 0) {
    Packet->HostAdapterStatus = EFI_EXT_SCSI_STATUS_HOST_ADAPTER_OK;
    Packet->TargetStatus      = EFI_EXT_SCSI_STATUS_TARGET_GOOD;
    return EFI_SUCCESS;
  }

Error:
  DEBUG((DEBUG_VERBOSE, "%a: dstat: %02X, sist0: %02X, sist1: %02X\n",
         __FUNCTION__, DStat, SIst0, SIst1));
  //
  // Update the request packet to reflect the status
  //
  if (*ScsiStatus != 0xFF) {
    Packet->TargetStatus    = *ScsiStatus;
  } else {
    Packet->TargetStatus    = EFI_EXT_SCSI_STATUS_TARGET_TASK_ABORTED;
  }
  Packet->HostAdapterStatus = EFI_EXT_SCSI_STATUS_HOST_ADAPTER_OTHER;
  Packet->InTransferLength  = 0;
  Packet->OutTransferLength = 0;
  Packet->SenseDataLength   = 0;

  return EFI_DEVICE_ERROR;
}

//
// EFI Ext SCSI Pass Thru Functions
//
STATIC
EFI_STATUS
EFIAPI
LsiScsiPassThru (
  IN EFI_EXT_SCSI_PASS_THRU_PROTOCOL                *This,
  IN UINT8                                          *Target,
  IN UINT64                                         Lun,
  IN OUT EFI_EXT_SCSI_PASS_THRU_SCSI_REQUEST_PACKET *Packet,
  IN EFI_EVENT                                      Event     OPTIONAL
  )
{
  EFI_STATUS   Status;
  LSI_SCSI_DEV *Dev;

  Dev = LSI_SCSI_FROM_PASS_THRU (This);
  Status = LsiScsiCheckRequest (Dev, *Target, Lun, Packet);
  if (EFI_ERROR (Status)) {
    return Status;
  }

  Status = LsiScsiProcessRequest (Dev, *Target, Lun, Packet);
  if (EFI_ERROR (Status)) {
    return Status;
  }

  return EFI_SUCCESS;
}

STATIC
EFI_STATUS
EFIAPI
LsiScsiGetNextTargetLun (
  IN EFI_EXT_SCSI_PASS_THRU_PROTOCOL *This,
  IN OUT UINT8                       **TargetPointer,
  IN OUT UINT64                      *Lun
  )
{
  LSI_SCSI_DEV *Dev;
  UINTN        Idx;
  UINT8        *Target;
  UINT16       LastTarget;

  //
  // the TargetPointer input parameter is unnecessarily a pointer-to-pointer
  //
  Target = *TargetPointer;

  //
  // Search for first non-0xFF byte. If not found, return first target & LUN.
  //
  for (Idx = 0; Idx < TARGET_MAX_BYTES && Target[Idx] == 0xFF; ++Idx)
    ;
  if (Idx == TARGET_MAX_BYTES) {
    SetMem (Target, TARGET_MAX_BYTES, 0x00);
    *Lun = 0;
    return EFI_SUCCESS;
  }

  CopyMem (&LastTarget, Target, sizeof LastTarget);

  //
  // increment (target, LUN) pair if valid on input
  //
  Dev = LSI_SCSI_FROM_PASS_THRU (This);
  if (LastTarget > Dev->MaxTarget || *Lun > Dev->MaxLun) {
    return EFI_INVALID_PARAMETER;
  }

  if (*Lun < Dev->MaxLun) {
    ++*Lun;
    return EFI_SUCCESS;
  }

  if (LastTarget < Dev->MaxTarget) {
    *Lun = 0;
    ++LastTarget;
    CopyMem (Target, &LastTarget, sizeof LastTarget);
    return EFI_SUCCESS;
  }

  return EFI_NOT_FOUND;
}

STATIC
EFI_STATUS
EFIAPI
LsiScsiBuildDevicePath (
  IN EFI_EXT_SCSI_PASS_THRU_PROTOCOL *This,
  IN UINT8                           *Target,
  IN UINT64                          Lun,
  IN OUT EFI_DEVICE_PATH_PROTOCOL    **DevicePath
  )
{
  UINT16           TargetValue;
  LSI_SCSI_DEV     *Dev;
  SCSI_DEVICE_PATH *ScsiDevicePath;

  if (DevicePath == NULL) {
    return EFI_INVALID_PARAMETER;
  }

  CopyMem (&TargetValue, Target, sizeof TargetValue);
  Dev = LSI_SCSI_FROM_PASS_THRU (This);
  if (TargetValue > Dev->MaxTarget || Lun > Dev->MaxLun || Lun > 0xFFFF) {
    return EFI_NOT_FOUND;
  }

  ScsiDevicePath = AllocatePool (sizeof *ScsiDevicePath);
  if (ScsiDevicePath == NULL) {
    return EFI_OUT_OF_RESOURCES;
  }

  ScsiDevicePath->Header.Type      = MESSAGING_DEVICE_PATH;
  ScsiDevicePath->Header.SubType   = MSG_SCSI_DP;
  ScsiDevicePath->Header.Length[0] = (UINT8)  sizeof *ScsiDevicePath;
  ScsiDevicePath->Header.Length[1] = (UINT8) (sizeof *ScsiDevicePath >> 8);
  ScsiDevicePath->Pun              = TargetValue;
  ScsiDevicePath->Lun              = (UINT16) Lun;

  *DevicePath = &ScsiDevicePath->Header;
  return EFI_SUCCESS;
}

EFI_STATUS
EFIAPI
LsiScsiGetTargetLun (
  IN  EFI_EXT_SCSI_PASS_THRU_PROTOCOL *This,
  IN  EFI_DEVICE_PATH_PROTOCOL        *DevicePath,
  OUT UINT8                           **TargetPointer,
  OUT UINT64                          *Lun
  )
{
  SCSI_DEVICE_PATH *ScsiDevicePath;
  LSI_SCSI_DEV     *Dev;
  UINT8            *Target;

  if (DevicePath == NULL || TargetPointer == NULL || *TargetPointer == NULL ||
      Lun == NULL) {
    return EFI_INVALID_PARAMETER;
  }

  if (DevicePath->Type    != MESSAGING_DEVICE_PATH ||
      DevicePath->SubType != MSG_SCSI_DP) {
    return EFI_UNSUPPORTED;
  }

  ScsiDevicePath = (SCSI_DEVICE_PATH *) DevicePath;
  Dev = LSI_SCSI_FROM_PASS_THRU (This);
  if (ScsiDevicePath->Pun > Dev->MaxTarget ||
      ScsiDevicePath->Lun > Dev->MaxLun) {
    return EFI_NOT_FOUND;
  }

  //
  // This device support 8 targets only, so it's enough to set the LSB
  // of Target.
  //
  Target = *TargetPointer;
  *Target = (UINT8)ScsiDevicePath->Pun;
  *Lun = ScsiDevicePath->Lun;

  return EFI_SUCCESS;
}

STATIC
EFI_STATUS
EFIAPI
LsiScsiResetChannel (
  IN EFI_EXT_SCSI_PASS_THRU_PROTOCOL *This
  )
{
  return EFI_UNSUPPORTED;
}

STATIC
EFI_STATUS
EFIAPI
LsiScsiResetTargetLun (
  IN EFI_EXT_SCSI_PASS_THRU_PROTOCOL *This,
  IN UINT8                           *Target,
  IN UINT64                          Lun
  )
{
  return EFI_UNSUPPORTED;
}

EFI_STATUS
EFIAPI
LsiScsiGetNextTarget (
  IN EFI_EXT_SCSI_PASS_THRU_PROTOCOL *This,
  IN OUT UINT8                       **TargetPointer
  )
{
  LSI_SCSI_DEV *Dev;
  UINTN        Idx;
  UINT8        *Target;
  UINT16       LastTarget;

  //
  // the TargetPointer input parameter is unnecessarily a pointer-to-pointer
  //
  Target = *TargetPointer;

  //
  // Search for first non-0xFF byte. If not found, return first target.
  //
  for (Idx = 0; Idx < TARGET_MAX_BYTES && Target[Idx] == 0xFF; ++Idx)
    ;
  if (Idx == TARGET_MAX_BYTES) {
    SetMem (Target, TARGET_MAX_BYTES, 0x00);
    return EFI_SUCCESS;
  }

  CopyMem (&LastTarget, Target, sizeof LastTarget);

  //
  // increment target if valid on input
  //
  Dev = LSI_SCSI_FROM_PASS_THRU (This);
  if (LastTarget > Dev->MaxTarget) {
    return EFI_INVALID_PARAMETER;
  }

  if (LastTarget < Dev->MaxTarget) {
    ++LastTarget;
    CopyMem (Target, &LastTarget, sizeof LastTarget);
    return EFI_SUCCESS;
  }

  return EFI_NOT_FOUND;
}

STATIC
VOID
EFIAPI
LsiScsiExitBoot (
  IN  EFI_EVENT Event,
  IN  VOID      *Context
  )
{
  LSI_SCSI_DEV *Dev;

  Dev = Context;
  DEBUG ((DEBUG_VERBOSE, "%a: Context=0x%p\n", __FUNCTION__, Context));
  LsiScsiReset (Dev);
}

//
// Driver Binding functions
//
STATIC
EFI_STATUS
EFIAPI
LsiScsiControllerSupported (
  IN EFI_DRIVER_BINDING_PROTOCOL            *This,
  IN EFI_HANDLE                             ControllerHandle,
  IN EFI_DEVICE_PATH_PROTOCOL               *RemainingDevicePath OPTIONAL
  )
{
  EFI_STATUS          Status;
  EFI_PCI_IO_PROTOCOL *PciIo;
  PCI_TYPE00          Pci;

  Status = gBS->OpenProtocol (
                  ControllerHandle,
                  &gEfiPciIoProtocolGuid,
                  (VOID **)&PciIo,
                  This->DriverBindingHandle,
                  ControllerHandle,
                  EFI_OPEN_PROTOCOL_BY_DRIVER
                  );
  if (EFI_ERROR (Status)) {
    return Status;
  }

  Status = PciIo->Pci.Read (
                        PciIo,
                        EfiPciIoWidthUint32,
                        0,
                        sizeof (Pci) / sizeof (UINT32),
                        &Pci
                        );
  if (EFI_ERROR (Status)) {
    goto Done;
  }

  if (Pci.Hdr.VendorId == LSI_LOGIC_PCI_VENDOR_ID &&
      Pci.Hdr.DeviceId == LSI_53C895A_PCI_DEVICE_ID) {
    Status = EFI_SUCCESS;
  } else {
    Status = EFI_UNSUPPORTED;
  }

Done:
  gBS->CloseProtocol (
         ControllerHandle,
         &gEfiPciIoProtocolGuid,
         This->DriverBindingHandle,
         ControllerHandle
         );
  return Status;
}

STATIC
EFI_STATUS
EFIAPI
LsiScsiControllerStart (
  IN EFI_DRIVER_BINDING_PROTOCOL            *This,
  IN EFI_HANDLE                             ControllerHandle,
  IN EFI_DEVICE_PATH_PROTOCOL               *RemainingDevicePath OPTIONAL
  )
{
  EFI_STATUS           Status;
  LSI_SCSI_DEV         *Dev;
  UINTN                Pages;
  UINTN                BytesMapped;

  Dev = AllocateZeroPool (sizeof (*Dev));
  if (Dev == NULL) {
    return EFI_OUT_OF_RESOURCES;
  }

  Dev->Signature = LSI_SCSI_DEV_SIGNATURE;

  Dev->MaxTarget = PcdGet8 (PcdLsiScsiMaxTargetLimit);
  Dev->MaxLun = PcdGet8 (PcdLsiScsiMaxLunLimit);
  Dev->StallPerPollUsec = PcdGet32 (PcdLsiScsiStallPerPollUsec);

  Status = gBS->OpenProtocol (
                  ControllerHandle,
                  &gEfiPciIoProtocolGuid,
                  (VOID **)&Dev->PciIo,
                  This->DriverBindingHandle,
                  ControllerHandle,
                  EFI_OPEN_PROTOCOL_BY_DRIVER
                  );
  if (EFI_ERROR (Status)) {
    goto FreePool;
  }

  Status = Dev->PciIo->Attributes (
                         Dev->PciIo,
                         EfiPciIoAttributeOperationGet,
                         0,
                         &Dev->OriginalPciAttributes
                         );
  if (EFI_ERROR (Status)) {
    goto CloseProtocol;
  }

  //
  // Enable I/O Space & Bus-Mastering
  //
  Status = Dev->PciIo->Attributes (
                         Dev->PciIo,
                         EfiPciIoAttributeOperationEnable,
                         (EFI_PCI_IO_ATTRIBUTE_IO |
                          EFI_PCI_IO_ATTRIBUTE_BUS_MASTER),
                         NULL
                         );
  if (EFI_ERROR (Status)) {
    goto CloseProtocol;
  }

  //
  // Signal device supports 64-bit DMA addresses
  //
  Status = Dev->PciIo->Attributes (
                         Dev->PciIo,
                         EfiPciIoAttributeOperationEnable,
                         EFI_PCI_IO_ATTRIBUTE_DUAL_ADDRESS_CYCLE,
                         NULL
                         );
  if (EFI_ERROR (Status)) {
    //
    // Warn user that device will only be using 32-bit DMA addresses.
    //
    // Note that this does not prevent the device/driver from working
    // and therefore we only warn and continue as usual.
    //
    DEBUG ((
      DEBUG_WARN,
      "%a: failed to enable 64-bit DMA addresses\n",
      __FUNCTION__
      ));
  }

  //
  // Create buffers for data transfer
  //
  Pages = EFI_SIZE_TO_PAGES (sizeof (*Dev->Dma));
  Status = Dev->PciIo->AllocateBuffer (
                         Dev->PciIo,
                         AllocateAnyPages,
                         EfiBootServicesData,
                         Pages,
                         (VOID **)&Dev->Dma,
                         EFI_PCI_ATTRIBUTE_MEMORY_CACHED
                         );
  if (EFI_ERROR (Status)) {
    goto RestoreAttributes;
  }

  BytesMapped = EFI_PAGES_TO_SIZE (Pages);
  Status = Dev->PciIo->Map (
                         Dev->PciIo,
                         EfiPciIoOperationBusMasterCommonBuffer,
                         Dev->Dma,
                         &BytesMapped,
                         &Dev->DmaPhysical,
                         &Dev->DmaMapping
                         );
  if (EFI_ERROR (Status)) {
    goto FreeBuffer;
  }

  if (BytesMapped != EFI_PAGES_TO_SIZE (Pages)) {
    Status = EFI_OUT_OF_RESOURCES;
    goto Unmap;
  }

  Status = LsiScsiReset (Dev);
  if (EFI_ERROR (Status)) {
    goto Unmap;
  }

  Status = gBS->CreateEvent (
                  EVT_SIGNAL_EXIT_BOOT_SERVICES,
                  TPL_CALLBACK,
                  &LsiScsiExitBoot,
                  Dev,
                  &Dev->ExitBoot
                  );
  if (EFI_ERROR (Status)) {
    goto UninitDev;
  }

  //
  // Host adapter channel, doesn't exist
  //
  Dev->PassThruMode.AdapterId = MAX_UINT32;
  Dev->PassThruMode.Attributes =
    EFI_EXT_SCSI_PASS_THRU_ATTRIBUTES_PHYSICAL |
    EFI_EXT_SCSI_PASS_THRU_ATTRIBUTES_LOGICAL;

  Dev->PassThru.Mode = &Dev->PassThruMode;
  Dev->PassThru.PassThru = &LsiScsiPassThru;
  Dev->PassThru.GetNextTargetLun = &LsiScsiGetNextTargetLun;
  Dev->PassThru.BuildDevicePath = &LsiScsiBuildDevicePath;
  Dev->PassThru.GetTargetLun = &LsiScsiGetTargetLun;
  Dev->PassThru.ResetChannel = &LsiScsiResetChannel;
  Dev->PassThru.ResetTargetLun = &LsiScsiResetTargetLun;
  Dev->PassThru.GetNextTarget = &LsiScsiGetNextTarget;

  Status = gBS->InstallProtocolInterface (
                  &ControllerHandle,
                  &gEfiExtScsiPassThruProtocolGuid,
                  EFI_NATIVE_INTERFACE,
                  &Dev->PassThru
                  );
  if (EFI_ERROR (Status)) {
    goto CloseExitBoot;
  }

  return EFI_SUCCESS;

CloseExitBoot:
  gBS->CloseEvent (Dev->ExitBoot);

UninitDev:
  LsiScsiReset (Dev);

Unmap:
  Dev->PciIo->Unmap (
                Dev->PciIo,
                Dev->DmaMapping
                );

FreeBuffer:
  Dev->PciIo->FreeBuffer (
                Dev->PciIo,
                Pages,
                Dev->Dma
                );

RestoreAttributes:
  Dev->PciIo->Attributes (
                Dev->PciIo,
                EfiPciIoAttributeOperationSet,
                Dev->OriginalPciAttributes,
                NULL
                );

CloseProtocol:
  gBS->CloseProtocol (
         ControllerHandle,
         &gEfiPciIoProtocolGuid,
         This->DriverBindingHandle,
         ControllerHandle
         );

FreePool:
  FreePool (Dev);

  return Status;
}

STATIC
EFI_STATUS
EFIAPI
LsiScsiControllerStop (
  IN EFI_DRIVER_BINDING_PROTOCOL            *This,
  IN EFI_HANDLE                            ControllerHandle,
  IN UINTN                                 NumberOfChildren,
  IN EFI_HANDLE                            *ChildHandleBuffer
  )
{
  EFI_STATUS                      Status;
  EFI_EXT_SCSI_PASS_THRU_PROTOCOL *PassThru;
  LSI_SCSI_DEV                    *Dev;

  Status = gBS->OpenProtocol (
                  ControllerHandle,
                  &gEfiExtScsiPassThruProtocolGuid,
                  (VOID **)&PassThru,
                  This->DriverBindingHandle,
                  ControllerHandle,
                  EFI_OPEN_PROTOCOL_GET_PROTOCOL // Lookup only
                  );
  if (EFI_ERROR (Status)) {
    return Status;
  }

  Dev = LSI_SCSI_FROM_PASS_THRU (PassThru);

  Status = gBS->UninstallProtocolInterface (
                  ControllerHandle,
                  &gEfiExtScsiPassThruProtocolGuid,
                  &Dev->PassThru
                  );
  if (EFI_ERROR (Status)) {
    return Status;
  }

  gBS->CloseEvent (Dev->ExitBoot);

  LsiScsiReset (Dev);

  Dev->PciIo->Unmap (
                Dev->PciIo,
                Dev->DmaMapping
                );

  Dev->PciIo->FreeBuffer (
                Dev->PciIo,
                EFI_SIZE_TO_PAGES (sizeof (*Dev->Dma)),
                Dev->Dma
                );

  Dev->PciIo->Attributes (
                Dev->PciIo,
                EfiPciIoAttributeOperationSet,
                Dev->OriginalPciAttributes,
                NULL
                );

  gBS->CloseProtocol (
         ControllerHandle,
         &gEfiPciIoProtocolGuid,
         This->DriverBindingHandle,
         ControllerHandle
         );

  FreePool (Dev);

  return Status;
}

STATIC
EFI_DRIVER_BINDING_PROTOCOL mLsiScsiDriverBinding = {
  &LsiScsiControllerSupported,
  &LsiScsiControllerStart,
  &LsiScsiControllerStop,
  0x10, // Version, must be in [0x10 .. 0xFFFFFFEF] for IHV-developed drivers
  NULL, // ImageHandle, to be overwritten by
        // EfiLibInstallDriverBindingComponentName2() in LsiScsiEntryPoint()
  NULL  // DriverBindingHandle, ditto
};

//
// Component Name
//
STATIC
EFI_UNICODE_STRING_TABLE mDriverNameTable[] = {
  { "eng;en", L"LSI 53C895A SCSI Controller Driver" },
  { NULL,     NULL                   }
};

STATIC
EFI_COMPONENT_NAME_PROTOCOL mComponentName;

EFI_STATUS
EFIAPI
LsiScsiGetDriverName (
  IN  EFI_COMPONENT_NAME_PROTOCOL *This,
  IN  CHAR8                       *Language,
  OUT CHAR16                      **DriverName
  )
{
  return LookupUnicodeString2 (
           Language,
           This->SupportedLanguages,
           mDriverNameTable,
           DriverName,
           (BOOLEAN)(This == &mComponentName) // Iso639Language
           );
}

EFI_STATUS
EFIAPI
LsiScsiGetDeviceName (
  IN  EFI_COMPONENT_NAME_PROTOCOL *This,
  IN  EFI_HANDLE                  DeviceHandle,
  IN  EFI_HANDLE                  ChildHandle,
  IN  CHAR8                       *Language,
  OUT CHAR16                      **ControllerName
  )
{
  return EFI_UNSUPPORTED;
}

STATIC
EFI_COMPONENT_NAME_PROTOCOL mComponentName = {
  &LsiScsiGetDriverName,
  &LsiScsiGetDeviceName,
  "eng" // SupportedLanguages, ISO 639-2 language codes
};

STATIC
EFI_COMPONENT_NAME2_PROTOCOL mComponentName2 = {
  (EFI_COMPONENT_NAME2_GET_DRIVER_NAME)     &LsiScsiGetDriverName,
  (EFI_COMPONENT_NAME2_GET_CONTROLLER_NAME) &LsiScsiGetDeviceName,
  "en" // SupportedLanguages, RFC 4646 language codes
};

//
// Entry point of this driver
//
EFI_STATUS
EFIAPI
LsiScsiEntryPoint (
  IN EFI_HANDLE       ImageHandle,
  IN EFI_SYSTEM_TABLE *SystemTable
  )
{
  return EfiLibInstallDriverBindingComponentName2 (
           ImageHandle,
           SystemTable,
           &mLsiScsiDriverBinding,
           ImageHandle, // The handle to install onto
           &mComponentName,
           &mComponentName2
           );
}
