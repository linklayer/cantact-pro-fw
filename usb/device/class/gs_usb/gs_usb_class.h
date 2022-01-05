// SPDX-License-Identifier: MIT
/*
 * gs_usb_class.h
 *
 *  Created on: Oct 27, 2018
 *      Author: eric
 */

#ifndef DEVICE_CLASS_GS_USB_CLASS_H_
#define DEVICE_CLASS_GS_USB_CLASS_H_

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#if defined(USB_DEVICE_CONFIG_EHCI) && (USB_DEVICE_CONFIG_EHCI > 0)
#define CONTROLLER_ID kUSB_ControllerEhci0
#define DATA_BUFF_SIZE HS_GS_USB_BULK_OUT_PACKET_SIZE

#endif
#if defined(USB_DEVICE_CONFIG_KHCI) && (USB_DEVICE_CONFIG_KHCI > 0)
#define CONTROLLER_ID kUSB_ControllerKhci0
#define DATA_BUFF_SIZE FS_GS_USB_BULK_OUT_PACKET_SIZE

#endif
#if defined(USB_DEVICE_CONFIG_LPCIP3511FS) && (USB_DEVICE_CONFIG_LPCIP3511FS > 0U)
#define CONTROLLER_ID kUSB_ControllerLpcIp3511Fs0
#define DATA_BUFF_SIZE FS_GS_USB_BULK_OUT_PACKET_SIZE

#endif

#if defined(USB_DEVICE_CONFIG_LPCIP3511HS) && (USB_DEVICE_CONFIG_LPCIP3511HS > 0U)
#define CONTROLLER_ID kUSB_ControllerLpcIp3511Hs0
#define DATA_BUFF_SIZE HS_GS_USB_BULK_OUT_PACKET_SIZE
#endif

#define USB_DEVICE_INTERRUPT_PRIORITY (3U)

/* Define the types for application */
typedef struct _usb_can_struct {
	usb_device_handle deviceHandle; /* USB device handle. */
	volatile uint8_t attach; /* A flag to indicate whether a usb device is attached. 1: attached, 0: not attached */
	uint8_t speed; /* Speed of USB device. USB_SPEED_FULL/USB_SPEED_LOW/USB_SPEED_HIGH.                 */
	uint8_t currentConfiguration; /* Current configuration value. */
	uint8_t hasSentState; /*!< 1: The device has primed the state in interrupt pipe, 0: Not primed the state. */
} usb_can_t;

usb_status_t USB_DeviceCallback(usb_device_handle handle, uint32_t event,
		void *param);
usb_status_t USB_ClassCallback(class_handle_t handle, uint32_t event,
		void *param);
void usb_send(uint8_t *buf, uint32_t len);
void gs_usb_init();

#endif /* DEVICE_CLASS_GS_USB_CLASS_H_ */
