/************************************************************************************//**
* \file         Source\com.c
* \brief        Bootloader communication interface source file.
* \ingroup      Core
* \internal
*----------------------------------------------------------------------------------------
*                          C O P Y R I G H T
*----------------------------------------------------------------------------------------
*   Copyright (c) 2011  by Feaser    http://www.feaser.com    All rights reserved
*
*----------------------------------------------------------------------------------------
*                            L I C E N S E
*----------------------------------------------------------------------------------------
* This file is part of OpenBLT. OpenBLT is free software: you can redistribute it and/or
* modify it under the terms of the GNU General Public License as published by the Free
* Software Foundation, either version 3 of the License, or (at your option) any later
* version.
*
* OpenBLT is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
* without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
* PURPOSE. See the GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with OpenBLT.
* If not, see <http://www.gnu.org/licenses/>.
*
* A special exception to the GPL is included to allow you to distribute a combined work 
* that includes OpenBLT without being obliged to provide the source code for any 
* proprietary components. The exception text is included at the bottom of the license
* file <license.html>.
* 
* \endinternal
****************************************************************************************/

/****************************************************************************************
* Include files
****************************************************************************************/
#include "boot.h"                                /* bootloader generic header          */
#if (BOOT_COM_CAN_ENABLE > 0)
  #include "can.h"                                    /* can driver module             */
#endif
#if (BOOT_COM_UART_ENABLE > 0)
  #include "uart.h"                                   /* uart driver module            */
#endif
#if (BOOT_COM_USB_ENABLE > 0)
  #include "usb.h"                                    /* usb driver module             */
#endif
#if (BOOT_COM_NET_ENABLE > 0)
  #include "net.h"                                    /* tcp/ip driver module          */
#endif


#if (BOOT_COM_ENABLE > 0)
/****************************************************************************************
* Local data declarations
****************************************************************************************/
/** \brief Holds the communication interface of the currently active interface. */
static tComInterfaceId comActiveInterface = COM_IF_OTHER;
#if (BOOT_DEBUGGING_UART2_ENABLE > 0)
static void InitDebugData(void);
#endif

#if (BOOT_DEBUGGING_UART2_ENABLE > 0)
static blt_int8u debugDataRUart[8];
#endif

/************************************************************************************//**
** \brief     Initializes the communication module including the hardware needed for 
**            the communication.
** \return    none
**
****************************************************************************************/
void ComInit(void)
{
#if (BOOT_DEBUGGING_UART2_ENABLE > 0)
  InitDebugData();
#endif
  /* initialize the XCP communication protocol */
  XcpInit();
#if (BOOT_COM_CAN_ENABLE > 0)
  /* initialize the CAN controller */
  CanInit();
  /* set it as active */
  comActiveInterface = COM_IF_CAN;
#endif
#if (BOOT_COM_UART_ENABLE > 0)
  /* initialize the UART interface */
  UartInit();
  /* set it as active */
  comActiveInterface = COM_IF_UART;
#endif
#if (BOOT_COM_BLUETOOTH_UART_ENABLE > 0)
  /* initialize the UART interface */
  BluetoothUartInit();
  /* set it as active */
  comActiveInterface = COM_IF_BLUETOOTH;
#endif
#if (BOOT_COM_USB_ENABLE > 0)
  /* initialize the USB interface */
  UsbInit();
  /* set it as active */
  comActiveInterface = COM_IF_USB;
#endif
#if (BOOT_COM_NET_ENABLE > 0)
  /* initialize the TCP/IP interface */
  NetInit();
  /* set it as active */
  comActiveInterface = COM_IF_NET;
#endif
} /*** end of ComInit ***/

#if (BOOT_DEBUGGING_UART2_ENABLE > 0)
/************************************************************************************//**
** \brief     Initializes the debugging data.
** \return    none
**
****************************************************************************************/
static void InitDebugData(void) {
  debugDataRUart[0] = 'r';
  debugDataRUart[1] = 'U';
  debugDataRUart[2] = 'A';
  debugDataRUart[3] = 'R';
  debugDataRUart[4] = 'T';
  debugDataRUart[5] = ':';
  debugDataRUart[6] = ' ';
  debugDataRUart[7] = 10;
} /*** end of InitDebugData ***/
#endif


/************************************************************************************//**
** \brief     Updates the communication module by checking if new data was received 
**            and submitting the request to process newly received data.
** \return    none
**
****************************************************************************************/
void ComTask(void)
{
  blt_int16s messageLength;
  /* make xcpCtoReqPacket static for runtime efficiency */
  static unsigned char xcpCtoReqPacket[BOOT_COM_RX_MAX_DATA];
 
#if (BOOT_COM_CAN_ENABLE > 0)
  messageLength = (blt_int16s)CanReceivePacket(&xcpCtoReqPacket[0]);
  if (messageLength > 0)
  {
    /* make this the active interface */
    comActiveInterface = COM_IF_CAN;
    /* process packet */
  #if (BOOT_GATE_ENABLE > 0)
    XcpPacketReceived(&xcpCtoReqPacket[0], messageLength, BLT_FALSE);
  #else
    XcpPacketReceived(&xcpCtoReqPacket[0], messageLength);
  #endif /* BOOT_GATE_ENABLE > 0 */
  }
#endif
#if (BOOT_COM_UART_ENABLE > 0)
  messageLength = (blt_int16s)UartReceivePacket(&xcpCtoReqPacket[0]);
  if (messageLength > 0)
  {
    /* make this the active interface */
    comActiveInterface = COM_IF_UART;
#if (BOOT_DEBUGGING_UART2_ENABLE > 0)
    /* send debugging data */
    BuildData(&debugDataRUart, &xcpCtoReqPacket[0], messageLength);
#endif
    /* process packet */
  #if (BOOT_GATE_ENABLE > 0)
    XcpPacketReceived(&xcpCtoReqPacket[0], messageLength, BLT_FALSE);
  #else
    XcpPacketReceived(&xcpCtoReqPacket[0], messageLength);
  #endif /* BOOT_GATE_ENABLE > 0 */
  }
#endif
#if (BOOT_COM_BLUETOOTH_UART_ENABLE > 0)
  messageLength = (blt_int16s)BluetoothUartReceivePacket(&xcpCtoReqPacket[0]);
  if (messageLength > 0)
  {
    //UartTransmitPacket(&xcpCtoReqPacket[0], messageLength);
    //UartTransmitPacket("\n", 1);
    /* make this the active interface */
    comActiveInterface = COM_IF_BLUETOOTH;
    /* process packet */
  #if (BOOT_GATE_ENABLE > 0)
    XcpPacketReceived(&xcpCtoReqPacket[0], messageLength, BLT_FALSE);
  #else
    XcpPacketReceived(&xcpCtoReqPacket[0], messageLength);
  #endif /* BOOT_GATE_ENABLE > 0 */
//  } else {
//    UartTransmitPacket("-\n", 2);
  }
#endif
#if (BOOT_COM_USB_ENABLE > 0)
  messageLength = (blt_int16s)UsbReceivePacket(&xcpCtoReqPacket[0]);
  if (messageLength > 0)
  {
    /* make this the active interface */
    comActiveInterface = COM_IF_USB;
    /* process packet */
  #if (BOOT_GATE_ENABLE > 0)
    XcpPacketReceived(&xcpCtoReqPacket[0], messageLength, BLT_FALSE);
  #else
    XcpPacketReceived(&xcpCtoReqPacket[0], messageLength);
  #endif /* BOOT_GATE_ENABLE > 0 */
  }
#endif
/* net conversation not impemented with length yet! */

//#if (BOOT_COM_NET_ENABLE > 0)
//  if (NetReceivePacket(&xcpCtoReqPacket[0]) == BLT_TRUE)
//  {
//    /* make this the active interface */
//    comActiveInterface = COM_IF_NET;
//    /* process packet */
//    XcpPacketReceived(&xcpCtoReqPacket[0]);
//  }
//#endif

} /*** end of ComTask ***/


/************************************************************************************//**
** \brief     Releases the communication module.
** \return    none
**
****************************************************************************************/
void ComFree(void)
{
#if (BOOT_COM_USB_ENABLE > 0)
  /* disconnect the usb device from the usb host */
  UsbFree();
#endif
} /*** end of ComFree ***/


/************************************************************************************//**
** \brief     Transmits the packet using the xcp transport layer.
** \param     data Pointer to the byte buffer with packet data.
** \param     len  Number of data bytes that need to be transmitted.
** \return    none
**
****************************************************************************************/
void ComTransmitPacket(blt_int8u *data, blt_int16u len)
{
#if (BOOT_COM_CAN_ENABLE > 0)
  /* transmit the packet. note that len is limited to 8 in the plausibility check,
   * so cast is okay.
   */
  if (comActiveInterface == COM_IF_CAN)
  {
    CanTransmitPacket(data, (blt_int8u)len, 0);
  }
#endif
#if (BOOT_COM_UART_ENABLE > 0)
  /* transmit the packet. note that len is limited to 255 in the plausibility check,
   * so cast is okay.
   */
  if (comActiveInterface == COM_IF_UART)
  {
    UartTransmitPacket(data, (blt_int8u)len);
  }
#endif
#if (BOOT_COM_BLUETOOTH_UART_ENABLE > 0)
  /* transmit the packet. note that len is limited to 255 in the plausibility check,

   * so cast is okay.
   */
  if (comActiveInterface == COM_IF_BLUETOOTH)
  {
    BluetoothUartTransmitPacket(data, (blt_int8u)len);
  }
#endif
#if (BOOT_COM_USB_ENABLE > 0)
  /* transmit the packet */
  if (comActiveInterface == COM_IF_USB)
  {
    UsbTransmitPacket(data, len);
  }
#endif
#if (BOOT_COM_NET_ENABLE > 0)
  if (comActiveInterface == COM_IF_NET)
  {
    /* transmit the packet */
    NetTransmitPacket(data, len);
  }
#endif

  /* send signal that the packet was transmitted */
  XcpPacketTransmitted();
} /*** end of ComTransmitPacket ***/


/************************************************************************************//**
** \brief     Obtains the maximum number of bytes that can be received on the specified
**            communication interface.
** \return    Maximum number of bytes that can be received.
**
****************************************************************************************/
blt_int16u ComGetActiveInterfaceMaxRxLen(void)
{
  blt_int16u result;
  
  /* filter on communication interface identifier */
  switch (comActiveInterface)
  {
    case COM_IF_UART:
      result = BOOT_COM_UART_RX_MAX_DATA;
      break;

    case COM_IF_BLUETOOTH:
      result = BOOT_COM_UART_RX_MAX_DATA;
      break;

    case COM_IF_CAN:
      result = BOOT_COM_CAN_RX_MAX_DATA;
      break;

    case COM_IF_USB:
      result = BOOT_COM_USB_RX_MAX_DATA;
      break;

    case COM_IF_NET:
      result = BOOT_COM_NET_RX_MAX_DATA;
      break;
      
    default:
      result = BOOT_COM_RX_MAX_DATA;
      break;
  }
  
  return result;
} /*** end of ComGetActiveInterfaceMaxRxLen ***/


/************************************************************************************//**
** \brief     Obtains the maximum number of bytes that can be transmitted on the 
**            specified communication interface.
** \return    Maximum number of bytes that can be received.
**
****************************************************************************************/
blt_int16u ComGetActiveInterfaceMaxTxLen(void)
{
  blt_int16u result;
  
  /* filter on communication interface identifier */
  switch (comActiveInterface)
  {
    case COM_IF_UART:
      result = BOOT_COM_UART_TX_MAX_DATA;
      break;

    case COM_IF_BLUETOOTH:
      result = BOOT_COM_UART_TX_MAX_DATA;
      break;

    case COM_IF_CAN:
      result = BOOT_COM_CAN_TX_MAX_DATA;
      break;

    case COM_IF_USB:
      result = BOOT_COM_USB_TX_MAX_DATA;
      break;

    case COM_IF_NET:
      result = BOOT_COM_NET_TX_MAX_DATA;
      break;
      
    default:
      result = BOOT_COM_TX_MAX_DATA;
      break;
  }
  
  return result;
} /*** end of ComGetActiveInterfaceMaxTxLen ***/


/************************************************************************************//**
** \brief     This function obtains the XCP connection state.
** \return    BLT_TRUE when an XCP connection is established, BLT_FALSE otherwise.
**
****************************************************************************************/
blt_bool ComIsConnected(void)
{
  return XcpIsConnected();
} /*** end of ComIsConnected ***/


#if (BOOTLOADER_OF_MAIN_DEVICE > 0)
/************************************************************************************//**
** \brief     This function obtains the XCP connection state.
** \return    BLT_TRUE when an XCP connection to main was established, BLT_FALSE otherwise.
**
****************************************************************************************/
blt_bool ComWasConnectedToMain(void)
{
  return XcpWasConnectedToMain();
} /*** end of ComWasConnectedToMain ***/
#endif /* BOOTLOADER_OF_MAIN_DEVICE > 0 */


#endif /* BOOT_COM_ENABLE > 0 */






#if (BOOT_COM_UART_ENABLE > 0)

/************************************************************************************//**
** \brief     This function transmitts the packet direct with UART.
** \param     data Pointer to the byte buffer with packet data.
** \param     len  Number of data bytes that need to be transmitted.
** \return    none.
**
****************************************************************************************/
void ComTransmitPacketDirect(blt_int8u *data, blt_int8u len) {
#if (BOOT_COM_BLUETOOTH_UART_ENABLE > 0)
  if (comActiveInterface == COM_IF_UART) {
#endif
    UartTransmitPacket(data, len);
#if (BOOT_COM_BLUETOOTH_UART_ENABLE > 0)
  } else {
    BluetoothUartTransmitPacket(data, len);
  }
#endif
  XcpPacketTransmitted();
} /*** end of ComTransmitPacketDirect ***/

#endif

/*********************************** end of com.c **************************************/
